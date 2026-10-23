#include "lora_beacon.h"
#include "app_config.h"
#include "main.h"
#include "sx127x.h"
#include "route_link.h"
#include "shore_link.h"
#include "event_queue.h"
#include "diag_frame.h"
#include "battery.h"
#include "lights.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Compact frame, at most 120 bytes (the station parses up to 127):
 * id=PHAO-01,10.762622,106.660172,r=2.1,p=-1.4,y=278.5,t=280.0,m=A,i=1,c=3,q=12,v=0,w=7:12:1,L=1,b=87,g=276.0,s=0.25
 * id=PHAO-01,NO_FIX,r=2.1,p=-1.4,y=278.5,t=-1.0,m=S,i=0,c=0,q=13,v=1,L=0,b=87
 * v is the number of people the Pi has reported since boot. m is S stopped, A the Pi is driving,
 * R driving home with the station lost.
 * The two decimal tokens without '=' remain lat/lon for backward compatibility. After v come the
 * optional fields, each only while there is room, most needed first: w=rev:have:pi once a route has
 * come over LoRa (route_link.h), H=1 while stopped from the dashboard (0 otherwise), L the lights,
 * b the battery %, g the GPS course and s the speed.
 *
 * Events (event_queue.h) go out in frames of their own, ahead of the beacon, until the station
 * acknowledges them; the station answers every frame (shore_link.h). */

#define TX_TIMEOUT_MS      1500U   /* SF9 BW125 takes ~0.4 s on air, over 1.5 s means a fault */
#define TX_POLL_PERIOD_MS  5U      /* do not poll TX_DONE every loop pass, it costs SPI time */
#define FIRST_TX_DELAY_MS  1000U
#define LORA_FRAME_SIZE    SX127X_MAX_PAYLOAD
#define RX_WINDOW_MS       1500U   /* the station answers ~50 ms after the beacon; its frame takes ~0.5 s */
#define ROUTE_BEACON_MS    1500U   /* beacon period while a route is coming in, one frame per beacon */

volatile lora_debug_t lora_debug;

#if LORA_ENABLED
static sx127x_t radio;
static const gps_state_t *gps_src;
static const bno055_euler_t *imu_src;
static const controller_t *control_src;
static uint32_t next_tx_ms;        /* next beacon */
static uint32_t event_due_ms;      /* next event frame, while one waits for its ACK */
static uint32_t tx_start_ms;
static uint32_t last_poll_ms;
static uint8_t victims;
static uint32_t rx_start_ms;
static route_link_t route;
static shore_link_t shore;
static event_queue_t events;
static diag_t diag;                /* the latest diagnostics from main.c (App/diag_frame.h) */
static bool diag_set;
static uint32_t next_diag_ms;
static bool listen_after_tx;       /* the station answers beacons and events, not diagnostics */

/* Round to the 0.1 deg the frame carries, then wrap: 359.96 would print as
 * "360.0", which the station rejects as out of range. */
static double heading_tenths(float deg)
{
    double tenths = fmod(round((double)deg * 10.0), 3600.0);
    if (tenths < 0.0) tenths += 3600.0;
    return tenths / 10.0;
}

/* Signed: the GPS interrupt can stamp a tick newer than now_ms. */
static bool gps_fix(uint32_t now_ms)
{
    return gps_src->valid && (int32_t)(now_ms - gps_src->last_rx_ms) <= (int32_t)LORA_GPS_STALE_MS;
}

/* Appends ",<field>" when it fits, otherwise leaves the frame as it was. */
static int append(char *buf, size_t size, int n, const char *field)
{
    if (field[0] == '\0') return n;
    int m = snprintf(buf + n, size - (size_t)n, ",%s", field);
    if (m > 0 && (size_t)(n + m) < size) return n + m;
    buf[n] = '\0';
    return n;
}

static uint16_t build_frame(char *buf, size_t size, uint32_t now_ms)
{
    bool fix = gps_fix(now_ms);
    /* Same freshness rule as Control_Tick: a good read within IMU_TIMEOUT_MS. */
    bool imu_ok = imu_src != NULL && control_src != NULL && control_src->last_imu_ms != 0U &&
                  (uint32_t)(now_ms - control_src->last_imu_ms) <= IMU_TIMEOUT_MS;
    double roll = imu_ok ? (double)imu_src->roll_deg : 0.0;
    double pitch = imu_ok ? (double)imu_src->pitch_deg : 0.0;
    double yaw = imu_ok ? heading_tenths(imu_src->heading_deg) : 0.0;
    double target = control_src != NULL && control_src->heading_target_valid
                  ? heading_tenths(control_src->heading_target_deg) : -1.0;
    char mode = control_src != NULL ? control_src->operating_mode : 'S';
    if (mode == 'A' && !ShoreLink_Ok(&shore, now_ms)) mode = 'R';
    unsigned int calib = imu_ok ? imu_src->calibration : 0U;
    int n;
    if (fix) {
        n = snprintf(buf, size,
                     "id=%s,%.6f,%.6f,r=%.1f,p=%.1f,y=%.1f,t=%.1f,m=%c,i=%u,c=%u,q=%lu,v=%u",
                     LORA_BUOY_ID, (double)gps_src->latitude_deg, (double)gps_src->longitude_deg,
                     roll, pitch, yaw, target, mode, imu_ok ? 1U : 0U, calib,
                     (unsigned long)lora_debug.seq, (unsigned int)victims);
    } else {
        n = snprintf(buf, size,
                     "id=%s,NO_FIX,r=%.1f,p=%.1f,y=%.1f,t=%.1f,m=%c,i=%u,c=%u,q=%lu,v=%u",
                     LORA_BUOY_ID, roll, pitch, yaw, target, mode, imu_ok ? 1U : 0U, calib,
                     (unsigned long)lora_debug.seq, (unsigned int)victims);
    }
    lora_debug.gps_valid = fix ? 1U : 0U;
    if (n < 0 || (size_t)n >= size) return 0U;   /* never send a cut frame */

    char field[24];
    RouteLink_BeaconField(&route, field, sizeof(field));
    n = append(buf, size, n, field);
    snprintf(field, sizeof(field), "H=%u", shore.hold ? 1U : 0U);
    n = append(buf, size, n, field);
    snprintf(field, sizeof(field), "L=%u", (unsigned)Lights_Get());
    n = append(buf, size, n, field);
    int percent = Battery_Percent();
    if (percent >= 0) {
        snprintf(field, sizeof(field), "b=%d", percent);
        n = append(buf, size, n, field);
    }
    if (fix) {
        snprintf(field, sizeof(field), "g=%.1f", heading_tenths(gps_src->course_deg));
        n = append(buf, size, n, field);
        snprintf(field, sizeof(field), "s=%.2f", (double)gps_src->speed_mps);
        n = append(buf, size, n, field);
    }
    return (uint16_t)n;
}

static bool boat_position(uint32_t now_ms, int32_t *lat_e6, int32_t *lon_e6)
{
    if (!gps_fix(now_ms)) return false;
    *lat_e6 = (int32_t)lround((double)gps_src->latitude_deg * 1e6);
    *lon_e6 = (int32_t)lround((double)gps_src->longitude_deg * 1e6);
    return true;
}

static void publish_link(void)
{
    lora_debug.events_pending = events.count;
    lora_debug.lights = Lights_Get();
    lora_debug.home_valid = shore.home_valid ? 1U : 0U;
    lora_debug.home_lat_e6 = shore.home_lat_e6;
    lora_debug.home_lon_e6 = shore.home_lon_e6;
    lora_debug.last_heard_ms = shore.last_heard_ms;
    lora_debug.events_dropped = events.dropped;
}
#endif /* LORA_ENABLED */

void LoraBeacon_Init(SPI_HandleTypeDef *hspi, const gps_state_t *gps,
                     const bno055_euler_t *imu, const controller_t *control)
{
    memset((void *)&lora_debug, 0, sizeof(lora_debug));
#if !LORA_ENABLED
    lora_debug.state = LORA_STATE_DISABLED;
    (void)hspi;
    (void)gps;
    (void)imu;
    (void)control;
#else
    gps_src = gps;
    imu_src = imu;
    control_src = control;
    RouteLink_Init(&route);
    ShoreLink_Init(&shore);
    EventQueue_Init(&events);
    radio.hspi = hspi;
    radio.nss_port = LORA_NSS_GPIO_Port;
    radio.nss_pin = LORA_NSS_Pin;
    radio.rst_port = LORA_RST_GPIO_Port;
    radio.rst_pin = LORA_RST_Pin;

    const sx127x_config_t cfg = {
        .frequency_hz = LORA_FREQUENCY_HZ,
        .spreading_factor = LORA_SF,
        .bandwidth_hz = LORA_BW_HZ,
        .coding_rate = LORA_CR,
        .sync_word = LORA_SYNC_WORD,
        .tx_power_dbm = LORA_TX_POWER_DBM,
        .payload_crc = true,
    };
    uint8_t version = 0U;
    sx127x_status_t st = SX127x_Init(&radio, &cfg, &version);
    lora_debug.version = version;
    lora_debug.last_error = (uint8_t)st;
    if (st != SX127X_OK) {
        lora_debug.state = LORA_STATE_INIT_FAIL;
        return;
    }
    lora_debug.init_ok = 1U;
    lora_debug.state = LORA_STATE_IDLE;
    next_tx_ms = HAL_GetTick() + FIRST_TX_DELAY_MS;
    next_diag_ms = next_tx_ms + LORA_BEACON_PERIOD_MS / 2U;   /* between the first two beacons */
#endif
}

void LoraBeacon_SetVictims(uint8_t count, uint32_t now_ms)
{
#if LORA_ENABLED
    if (count == victims) return;
    victims = count;
    next_tx_ms = now_ms;   /* a frame on air finishes first, then this one goes out */
#else
    (void)count;
    (void)now_ms;
#endif
}

void LoraBeacon_ReportVictim(bool has_pos, int32_t lat_e6, int32_t lon_e6, uint8_t risk, uint8_t confidence_pct, uint32_t now_ms)
{
#if LORA_ENABLED
    EventQueue_PushVictim(&events, has_pos, lat_e6, lon_e6, risk, confidence_pct, victims);
    event_due_ms = now_ms;
    publish_link();
#else
    (void)has_pos; (void)lat_e6; (void)lon_e6; (void)risk; (void)confidence_pct; (void)now_ms;
#endif
}

void LoraBeacon_ReportTilt(float roll_deg, float pitch_deg, uint32_t now_ms)
{
#if LORA_ENABLED
    int32_t lat = 0, lon = 0;
    bool has_pos = boat_position(now_ms, &lat, &lon);
    EventQueue_PushTilt(&events, has_pos, lat, lon, roll_deg, pitch_deg);
    event_due_ms = now_ms;
    publish_link();
#else
    (void)roll_deg; (void)pitch_deg; (void)now_ms;
#endif
}

void LoraBeacon_ReportConfirm(uint16_t presses, uint32_t now_ms)
{
#if LORA_ENABLED
    int32_t lat = 0, lon = 0;
    bool has_pos = boat_position(now_ms, &lat, &lon);
    EventQueue_PushConfirm(&events, has_pos, lat, lon, presses);
    event_due_ms = now_ms;
    publish_link();
#else
    (void)presses; (void)now_ms;
#endif
}

void LoraBeacon_Tick(uint32_t now_ms)
{
#if LORA_ENABLED
    if (lora_debug.state == LORA_STATE_IDLE) {
        char frame[LORA_FRAME_SIZE];
        uint16_t len = 0U;
        if (EventQueue_Pending(&events) && (int32_t)(now_ms - event_due_ms) >= 0) {
            /* An event first; it goes out again after LORA_EVENT_RETRY_MS until the station acknowledges it. */
            len = (uint16_t)EventQueue_Frame(&events, LORA_BUOY_ID, frame, sizeof(frame));
            event_due_ms = now_ms + LORA_EVENT_RETRY_MS;
            if (len > 0U) lora_debug.event_tx++;
            publish_link();
        }
        listen_after_tx = true;
        if (len == 0U && (int32_t)(now_ms - next_tx_ms) >= 0) {
            len = build_frame(frame, sizeof(frame), now_ms);
            next_tx_ms = now_ms + (RouteLink_Receiving(&route) ? ROUTE_BEACON_MS : LORA_BEACON_PERIOD_MS);
            lora_debug.seq++;
        } else if (len == 0U && diag_set && (int32_t)(now_ms - next_diag_ms) >= 0 && !RouteLink_Receiving(&route)) {
            /* Diagnostics between two beacons, not while a route comes in; nobody answers them. */
            len = (uint16_t)DiagFrame_Build(frame, sizeof(frame), LORA_BUOY_ID, &diag);
            next_diag_ms = now_ms + LORA_DIAG_PERIOD_MS;
            listen_after_tx = false;
            if (len > 0U) lora_debug.diag_tx++;
        }
        if (len == 0U) return;
        if (SX127x_StartTx(&radio, (const uint8_t *)frame, (uint8_t)len)) {
            lora_debug.last_len = len;
            tx_start_ms = now_ms;
            last_poll_ms = now_ms;
            lora_debug.state = LORA_STATE_TX;
        } else {
            SX127x_Standby(&radio);
            lora_debug.tx_fail++;
        }
    } else if (lora_debug.state == LORA_STATE_TX) {
        if ((uint32_t)(now_ms - last_poll_ms) < TX_POLL_PERIOD_MS) return;
        last_poll_ms = now_ms;
        if (SX127x_TxDone(&radio)) {
            lora_debug.tx_count++;
            lora_debug.last_tx_ms = now_ms;
            /* Listen for the station's answer; without it the radio rests until the next frame. */
            rx_start_ms = now_ms;
            if (!listen_after_tx) { SX127x_Standby(&radio); lora_debug.state = LORA_STATE_IDLE; }
            else lora_debug.state = SX127x_StartRx(&radio) ? LORA_STATE_RX : LORA_STATE_IDLE;
        } else if ((uint32_t)(now_ms - tx_start_ms) > TX_TIMEOUT_MS) {
            SX127x_Standby(&radio);
            lora_debug.tx_fail++;
            lora_debug.state = LORA_STATE_IDLE;
        }
    } else if (lora_debug.state == LORA_STATE_RX) {
        if ((uint32_t)(now_ms - last_poll_ms) < TX_POLL_PERIOD_MS) return;
        last_poll_ms = now_ms;
        uint8_t frame[LORA_FRAME_SIZE + 1U];
        uint8_t len = 0U;
        bool ours = false;
        if (SX127x_ReadPacket(&radio, frame, LORA_FRAME_SIZE, &len)) {
            frame[len] = '\0';
            bool was_receiving = RouteLink_Receiving(&route);
            uint16_t eseq = 0U;
            if (RouteLink_OnFrame(&route, (const char *)frame, LORA_BUOY_ID)) {
                ours = true;
                ShoreLink_Heard(&shore, now_ms);
                if (RouteLink_Complete(&route) && (was_receiving || route.total == 0U))
                    next_tx_ms = now_ms;   /* tell the station at once that the route is complete */
            } else if (ShoreLink_OnFrame(&shore, (const char *)frame, LORA_BUOY_ID, now_ms, &eseq)) {
                ours = true;
                lora_debug.ack_count++;
                Lights_Set(shore.lights);
                lora_debug.hold = shore.hold ? 1U : 0U;
                if (EventQueue_Ack(&events, eseq) && EventQueue_Pending(&events))
                    event_due_ms = now_ms;   /* the next event at once */
            }
            publish_link();
        }
        /* One answer per frame: stop listening once ours is in, or when the window closes. The
           station answers every buoy on the one channel: an ACK for another buoy, sent just before
           ours, used to close the window and the RTE behind it was lost, beacon after beacon while
           the two buoys kept the same rhythm. The radio stays in RX_CONTINUOUS meanwhile. */
        if (ours || (uint32_t)(now_ms - rx_start_ms) > RX_WINDOW_MS) {
            SX127x_Standby(&radio);
            lora_debug.state = LORA_STATE_IDLE;
        }
    }
    lora_debug.shore_ok = ShoreLink_Ok(&shore, now_ms) ? 1U : 0U;
#else
    (void)now_ms;
#endif
}

bool LoraBeacon_Hold(void)
{
#if LORA_ENABLED
    return shore.hold;
#else
    return false;
#endif
}

bool LoraBeacon_ShoreOk(uint32_t now_ms)
{
#if LORA_ENABLED
    return ShoreLink_Ok(&shore, now_ms);
#else
    (void)now_ms;
    return true;
#endif
}

size_t LoraBeacon_LinkLine(uint16_t seq, uint32_t now_ms, char *buf, size_t size)
{
#if LORA_ENABLED
    return ShoreLink_PiLine(&shore, seq, now_ms, buf, size);
#else
    shore_link_t never;
    ShoreLink_Init(&never);
    return ShoreLink_PiLine(&never, seq, now_ms, buf, size);
#endif
}

void LoraBeacon_SetDiag(const diag_t *latest)
{
#if LORA_ENABLED
    diag = *latest;
    diag_set = true;
#else
    (void)latest;
#endif
}

void LoraBeacon_SetPiRouteRev(uint16_t rev)
{
#if LORA_ENABLED
    RouteLink_SetPiRev(&route, rev);
#else
    (void)rev;
#endif
}

bool LoraBeacon_NextPiRouteLine(uint16_t seq, uint32_t now_ms, char *buf, size_t size)
{
#if LORA_ENABLED
    return RouteLink_NextPiLine(&route, seq, now_ms, buf, size);
#else
    (void)seq; (void)now_ms; (void)buf; (void)size;
    return false;
#endif
}
