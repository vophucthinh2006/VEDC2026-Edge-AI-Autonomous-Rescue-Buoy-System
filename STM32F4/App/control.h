#pragma once

#include "actuators.h"
#include "imu.h"
#include "gps_nmea.h"
#include "ibus.h"
#include "uart_protocol.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool armed;
    bool arm_switch_released;   /* arm switch seen off since the last arming */
    bool manual_ready;          /* speed stick centred since entering MANUAL */
    bool overturned;
    bool motor_fault;
    bool estop;
    float heading_integral;
    uint32_t last_imu_ms;
} controller_t;

void Control_Init(controller_t *control);
void Camera_Tick(const pi_command_t *pi);
void Control_Tick(controller_t *control, const pi_command_t *pi, const ibus_state_t *rc, const bno055_euler_t *imu, uint32_t now_ms);
