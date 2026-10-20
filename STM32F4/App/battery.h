#pragma once

/* Battery voltage, current and charge left, from the INA226 on I2C1 (shared with the BNO055: both
 * are read from the main loop, one after the other, never at the same time). The charge is read off
 * the resting voltage of a LiPo pack of BATTERY_CELLS cells, so it dips while the motors pull hard;
 * a slow filter keeps it from jumping. */

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

/* Read live over ST-LINK (scripts/stm32/test_battery.bat). Keep the layout in step with it. */
typedef struct {
    float volts;               /* filtered */
    float amps;                /* filtered, needs INA226_SHUNT_OHM to match the module */
    int8_t percent;            /* -1 unknown */
    uint8_t ok;                /* INA226 found and answering */
    uint16_t fails;            /* failed reads since boot */
    uint32_t reads;
    uint8_t address;           /* 7-bit address the INA226 answered on, 0 = not found */
} battery_debug_t;

extern volatile battery_debug_t battery_debug;

void Battery_Init(I2C_HandleTypeDef *hi2c);
/* Call every control tick; it reads the INA226 every BATTERY_READ_PERIOD_MS. */
void Battery_Tick(uint32_t now_ms);
float Battery_Volts(void);         /* 0 unknown */
int Battery_Percent(void);         /* -1 unknown */

/* Charge 0..100 of one LiPo cell at volts_per_cell (pure, for the host test). */
int Battery_CellPercent(float volts_per_cell);
