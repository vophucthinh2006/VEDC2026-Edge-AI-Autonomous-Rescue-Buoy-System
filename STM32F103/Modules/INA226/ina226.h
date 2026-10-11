#pragma once

/* INA226 current and bus-voltage monitor on I2C. It knows nothing about the App layer: the
 * address and the calibration value come in through the handle. */

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address;            /* 7-bit */
    uint16_t calibration;       /* 0.00512 / (current LSB in A * shunt in ohm) */
    uint16_t current_lsb_ua;    /* microamps per count of the current register */
} ina226_t;

/* Checks the manufacturer ID (0x5449) and writes the calibration. False: no chip or a bus error. */
bool INA226_Init(ina226_t *dev);
/* Bus voltage in mV (1.25 mV per count) and current in mA, positive from IN+ to IN-. */
bool INA226_Read(ina226_t *dev, uint16_t *bus_mv, int32_t *current_ma);
