#pragma once

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float heading_deg;
    float roll_deg;
    float pitch_deg;
    uint8_t calibration; /* 0..3: system calibration nibble */
    bool valid;
} bno055_euler_t;

bool BNO055_Init(I2C_HandleTypeDef *hi2c);
bool BNO055_ReadEuler(I2C_HandleTypeDef *hi2c, bno055_euler_t *out);
