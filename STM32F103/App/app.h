#pragma once

#include "main.h"
#include <stdint.h>

/* State at a glance, read live over ST-LINK while the board runs. Widest fields first, so the
   layout has no padding. */
typedef struct {
    uint32_t lora_tx;          /* beacons fully sent */
    uint32_t lora_fail;        /* beacons dropped on an SPI error or a timeout */
    uint32_t lora_seq;
    int32_t current_ma;
    uint16_t bus_mv;
    uint16_t servo_us;
    uint16_t pi_rx_ok;         /* lines from the Pi accepted */
    uint16_t pi_rx_bad;        /* lines from the Pi dropped */
    uint16_t ibus_polls;       /* polls from the receiver answered */
    uint8_t pi_ok;             /* HBT within PI_LINK_TIMEOUT_MS */
    uint8_t victims;
    uint8_t alarm;
    uint8_t estop;
    uint8_t ina_ok;
    uint8_t lora_version;      /* REG_VERSION read, expected 0x12 */
    uint8_t lora_state;        /* 0 off, 1 init failed, 2 idle, 3 transmitting */
} helper_debug_t;

extern volatile helper_debug_t helper_debug;

void App_Init(void);
/* Every pass of the main loop: it keeps its own periods. */
void App_Tick(void);
/* From HAL_UART_RxCpltCallback and HAL_UART_ErrorCallback in main.c. */
void App_UartRxComplete(UART_HandleTypeDef *huart);
void App_UartError(UART_HandleTypeDef *huart);
