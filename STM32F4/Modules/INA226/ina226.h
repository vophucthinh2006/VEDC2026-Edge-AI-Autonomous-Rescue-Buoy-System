#pragma once

/* TI INA226 bus voltage / current monitor on I2C (datasheet SBOS547).
 * Bus voltage is 1.25 mV a bit; the current register is scaled by the calibration register, which
 * INA226_Init derives from the shunt resistance and the largest current expected. */

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

#define INA226_ADDR_DEFAULT  0x40U   /* A0 = A1 = GND */

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address;           /* 7-bit */
    float current_lsb_a;       /* set by INA226_Init */
} ina226_t;

/* Checks the manufacturer ID, then sets 16-sample averaging, 1.1 ms conversions, continuous shunt
 * and bus measurement, and the calibration for shunt_ohm / max_current_a. */
bool INA226_Init(ina226_t *dev, I2C_HandleTypeDef *hi2c, uint8_t address, float shunt_ohm, float max_current_a);

/* Latest bus voltage (V) and current (A, positive into IN+). */
bool INA226_Read(ina226_t *dev, float *volts, float *amps);
