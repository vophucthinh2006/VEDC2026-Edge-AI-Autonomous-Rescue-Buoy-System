#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

/* I2C3 (PA8 SCL, PC9 SDA) is the BNO055 bus; I2C1 (PB8 SCL, PB9 SDA) is the INA226 battery bus.
 * A slave cut off mid-byte (a brown-out or motor noise) can hold SDA low and leave the F4's I2C BUSY.
 *
 * I2cBus_Ready, before the tick's transfers: true when the bus is idle. When it is not, the bus is
 * recovered independently (at most every I2C_RECOVER_PERIOD_MS) and the tick skips its transfers. */

typedef struct {
    uint16_t busy_ticks;       /* ticks that found the bus busy */
    uint16_t recoveries;
    uint16_t sda_stuck;        /* recoveries after which SDA was still low */
    uint16_t init_fails;       /* HAL_I2C_Init failed after a recovery */
} i2c_bus_debug_t;

extern volatile i2c_bus_debug_t i2c_bus_debug;

bool I2cBus_Ready(I2C_HandleTypeDef *hi2c, uint32_t now_ms);
/* Clock out a stuck slave (up to 9 SCL pulses), send a STOP, then reset and start the peripheral again. */
void I2cBus_Recover(I2C_HandleTypeDef *hi2c);
