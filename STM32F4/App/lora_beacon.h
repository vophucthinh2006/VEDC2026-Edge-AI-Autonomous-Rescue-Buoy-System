#pragma once

/* Sends GPS and attitude to the shore station every LORA_BEACON_PERIOD_MS, and events (a person,
 * overturned, the confirm button) at once in frames of their own (App/event_queue.h). After each
 * frame it listens for the station's answer: a route frame (App/route_link.c) or an ACK with the
 * lights and the station's position (App/shore_link.c); docs/lora-route-downlink.md.
 * Non-blocking: each loop pass does a few short SPI transfers, so the control loop is unaffected. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "gps_nmea.h"
#include "imu.h"
#include "control.h"
#include "diag_frame.h"

/* Beacon state, read over OpenOCD (Tools/lora_monitor.py). Keep the layout in step with that file. */
typedef enum {
    LORA_STATE_DISABLED = 0,   /* LORA_ENABLED = 0 */
    LORA_STATE_INIT_FAIL = 1,  /* no module or SPI error, see last_error */
    LORA_STATE_IDLE = 2,       /* waiting for the next period */
    LORA_STATE_TX = 3,         /* transmitting */
    LORA_STATE_RX = 4          /* listening for the station's answer after a beacon */
} lora_state_t;

typedef struct {
    uint8_t version;           /* REG_VERSION read, expected 0x12 */
    uint8_t init_ok;
    uint8_t state;             /* lora_state_t */
    uint8_t last_error;        /* sx127x_status_t of the Init call */
    uint32_t tx_count;         /* packets fully sent */
    uint32_t tx_fail;          /* packets dropped on SPI error or timeout */
    uint32_t seq;              /* sequence number of the next packet */
    uint32_t last_tx_ms;
    uint16_t last_len;
    uint16_t gps_valid;        /* whether the last frame carried a position */
    /* Shore link and events; appended so the fields above keep their offsets. */
    uint32_t ack_count;        /* ACK frames taken from the station */
    uint32_t event_tx;         /* event frames sent */
    uint32_t events_dropped;   /* events never acknowledged */
    uint32_t last_heard_ms;    /* last frame of the station */
    int32_t home_lat_e6;       /* the station's position, the way home */
    int32_t home_lon_e6;
    uint8_t events_pending;
    uint8_t shore_ok;          /* 0: the station has been silent for SHORE_LINK_TIMEOUT_MS */
    uint8_t lights;            /* relays driven, bit 0 PC3, bit 1 PC2 */
    uint8_t home_valid;
    uint8_t hold;              /* 1: STOP from the dashboard, motors off */
    uint32_t diag_tx;          /* diagnostics frames sent (App/diag_frame.h); appended last */
} lora_debug_t;

extern volatile lora_debug_t lora_debug;

void LoraBeacon_Init(SPI_HandleTypeDef *hspi, const gps_state_t *gps,
                     const bno055_euler_t *imu, const controller_t *control);
void LoraBeacon_Tick(uint32_t now_ms);
/* Number of people the Pi has reported. Every frame carries it, so one lost
 * packet does not lose the alert; a new count is sent without waiting for the period. */
void LoraBeacon_SetVictims(uint8_t count, uint32_t now_ms);

/* Events for the shore, sent at once and again until acknowledged. A person at the position the
 * Pi reckoned (has_pos false: none), risk 0 safe .. 3 critical; overturned and the confirm button
 * with the boat's own GPS. */
void LoraBeacon_ReportVictim(bool has_pos, int32_t lat_e6, int32_t lon_e6, uint8_t risk, uint8_t confidence_pct, uint32_t now_ms);
void LoraBeacon_ReportTilt(float roll_deg, float pitch_deg, uint32_t now_ms);
void LoraBeacon_ReportConfirm(uint16_t presses, uint32_t now_ms);

/* The shore station: false once it has been silent for SHORE_LINK_TIMEOUT_MS. The payload of the
 * $LNK line for the Pi (App/shore_link.h). */
bool LoraBeacon_ShoreOk(uint32_t now_ms);
/* STOP from the dashboard (bit 2 of the station's ACK): the motors stay off until released. Off at
 * boot until the first ACK says otherwise. */
bool LoraBeacon_Hold(void);
size_t LoraBeacon_LinkLine(uint16_t seq, uint32_t now_ms, char *buf, size_t size);

/* Route from the dashboard, for the Pi: the revision the Pi's $HBT reports, and the next $RTE line
 * payload to send (false: nothing now). Call NextPiRouteLine once per telemetry period. */
void LoraBeacon_SetPiRouteRev(uint16_t rev);
/* The latest diagnostics (App/diag_frame.h), sent in a frame of their own every LORA_DIAG_PERIOD_MS. */
void LoraBeacon_SetDiag(const diag_t *latest);
bool LoraBeacon_NextPiRouteLine(uint16_t seq, uint32_t now_ms, char *buf, size_t size);
