#pragma once

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    TIM_HandleTypeDef *tim3;
    TIM_HandleTypeDef *tim4;
} actuator_driver_t;

void Actuators_Init(actuator_driver_t *driver, TIM_HandleTypeDef *tim3, TIM_HandleTypeDef *tim4);
void Actuators_Stop(const actuator_driver_t *driver);
void Actuators_SetAuto(const actuator_driver_t *driver, float throttle_0_to_1, float yaw_correction);
void Actuators_SetManual(const actuator_driver_t *driver, uint16_t throttle_us, uint16_t yaw_us);
