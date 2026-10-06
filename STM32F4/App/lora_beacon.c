#include "lora_beacon.h"
#include "app_config.h"
#include "main.h"
#include "sx127x.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Compact frame, always below 100 bytes because the station keeps 100 bytes:
 * id=PHAO-01,10.762622,106.660172,r=2.1,p=-1.4,y=278.5,t=280.0,m=A,i=1,c=3,q=12,v=0
 * id=PHAO-01,NO_FIX,r=2.1,p=-1.4,y=278.5,t=-1.0,m=S,i=0,c=0,q=13,v=1
 * v is the number of people the Pi has reported since boot.
 * The two decimal tokens without '=' remain lat/lon for backward compatibility. */

#define TX_TIMEOUT_MS      1500U   /* SF9 BW125 takes ~0.4 s on air, over 1.5 s means a fault */
#define TX_POLL_PERIOD_MS  5U      /* do not poll TX_DONE every loop pass, it costs SPI time */
#define FIRST_TX_DELAY_MS  1000U
#define LORA_FRAME_SIZE    100U

volatile lora_debug_t lora_debug;

#if LORA_ENABLED
static sx127x_t radio;
static const gps_state_t *gps_src;
static const bno055_euler_t *imu_src;
static const controller_t *control_src;
static uint32_t next_tx_ms;
static uint32_t tx_start_ms;
static uint32_t last_poll_ms;
static uint8_t victims;

/* Round to the 0.1 deg the frame carries, then wrap: 359.96 would print as
 * "360.0", which the station rejects as out of range. */
static double heading_tenths(float deg)
{
    double tenths = fmod(round((double)deg * 10.0), 3600.0);
    if (tenths < 0.0) tenths += 3600.0;
    return tenths / 10.0;
}

static uint16_t build_frame(char *buf, size_t size, uint32_t now_ms)
{
    /* Signed: the GPS interrupt can stamp a tick newer than now_ms. */
    bool fix = gps_src->valid && (int32_t)(now_ms - gps_src->last_rx_ms) <= (int32_t)LORA_GPS_STALE_MS;
    /* Same freshness rule as Control_Tick: a good read within IMU_TIMEOUT_MS. */
    bool imu_ok = imu_src != NULL && control_src != NULL && control_src->last_imu_ms != 0U &&
                  (uint32_t)(now_ms - control_src->last_imu_ms) <= IMU_TIMEOUT_MS;
    double roll = imu_ok ? (double)imu_src->roll_deg : 0.0;
    double pitch = imu_ok ? (double)imu_src->pitch_deg : 0.0;
    double yaw = imu_ok ? heading_tenths(imu_src->heading_deg) : 0.0;
    double target = control_src != NULL && control_src->heading_target_valid
                  ? heading_tenths(control_src->heading_target_deg) : -1.0;
    char mode = control_src != NULL ? control_src->operating_mode : 'S';
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
    if (n < 0) return 0U;
    return (uint16_t)((size_t)n >= size ? size - 1U : (size_t)n);
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

void LoraBeacon_Tick(uint32_t now_ms)
{
#if LORA_ENABLED
    if (lora_debug.state == LORA_STATE_IDLE) {
        if ((int32_t)(now_ms - next_tx_ms) < 0) return;

        char frame[LORA_FRAME_SIZE];
        uint16_t len = build_frame(frame, sizeof(frame), now_ms);
        next_tx_ms = now_ms + LORA_BEACON_PERIOD_MS;
        lora_debug.seq++;
        if (len > 0U && SX127x_StartTx(&radio, (const uint8_t *)frame, (uint8_t)len)) {
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
            lora_debug.state = LORA_STATE_IDLE;
        } else if ((uint32_t)(now_ms - tx_start_ms) > TX_TIMEOUT_MS) {
            SX127x_Standby(&radio);
            lora_debug.tx_fail++;
            lora_debug.state = LORA_STATE_IDLE;
        }
    }
#else
    (void)now_ms;
#endif
}
