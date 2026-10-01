#pragma once

/* Sends GPS and attitude to the shore station every LORA_BEACON_PERIOD_MS.
 * Non-blocking: each loop pass does a few short SPI transfers, so the control loop is unaffected. */

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "gps_nmea.h"
#include "imu.h"
#include "control.h"

/* Beacon state, read over OpenOCD (Tools/lora_monitor.py). Keep the layout in step with that file. */
typedef enum {
    LORA_STATE_DISABLED = 0,   /* LORA_ENABLED = 0 */
    LORA_STATE_INIT_FAIL = 1,  /* no module or SPI error, see last_error */
    LORA_STATE_IDLE = 2,       /* waiting for the next period */
    LORA_STATE_TX = 3          /* transmitting */
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
} lora_debug_t;

extern volatile lora_debug_t lora_debug;

void LoraBeacon_Init(SPI_HandleTypeDef *hspi, const gps_state_t *gps,
                     const bno055_euler_t *imu, const controller_t *control);
void LoraBeacon_Tick(uint32_t now_ms);
