#pragma once

#include "actuators.h"
#include "imu.h"
#include "gps_nmea.h"
#include "uart_protocol.h"
#include "heading_controller.h"
#include "confirm_button.h"
#include "front_motor.h"
#include <stdbool.h>
#include <stdint.h>

/* Whether the Pi's heading is being held right now. */
typedef enum {
    HEADING_OFF = 0,
    HEADING_AUTO = 1         /* target from the Pi NAV packet */
} heading_state_t;

typedef struct {
    bool armed;                 /* safe to drive: fresh IMU (or BENCH_IGNORE_IMU), not overturned */
    bool overturned;
    bool motor_fault;
    bool estop;                 /* always false: PE4 is the confirm button (App/confirm_button.c) */
    confirm_button_t confirm;   /* presses of the victim's confirm button, sent in $SYS */
    heading_controller_t heading;
    float heading_target_deg;
    float heading_cmd;            /* PID output u, -1..1, positive turns the bow right */
    bool heading_target_valid;
    uint8_t heading_state;        /* heading_state_t */
    char operating_mode;          /* 'S' stopped, 'A' the Pi is driving */
    bool hold;                    /* STOP from the dashboard, set by main.c from the LoRa ACK */
    front_motor_t front;          /* the front motor: pushes with the rear ones, brakes (App/front_motor.c) */
    uint32_t last_imu_ms;
} controller_t;

/* Heading loop at a glance, read live over ST-LINK by Tools/heading_monitor.py.
   Floats first, then bytes: keep the layout in step with that script. */
typedef struct {
    float target_deg;
    float yaw_deg;
    float yaw_rate_dps;    /* raw from the IMU, positive clockwise */
    float error_deg;
    float p, i, d;
    float u;
    float steer;
    float rear_left;
    float rear_right;
    float roll_deg;
    float pitch_deg;
    uint16_t imu_fail;     /* control ticks with a failed IMU read since boot */
    uint8_t state;         /* heading_state_t */
    uint8_t imu_ok;
    uint8_t block;         /* drive_block_t: why the motors are not driven */
    uint8_t armed;
    uint8_t arm_block;     /* arm_block_t: why the boat is not armed */
    uint8_t last_disarm;   /* arm_block_t that ended the last arming, 0 = none yet */
} heading_debug_t;

typedef enum {
    ARM_BLOCK_NONE = 0,
    ARM_BLOCK_IMU = 1,          /* no fresh BNO055 reading */
    ARM_BLOCK_OVERTURNED = 2    /* pitch or roll past MAX_PITCH_DEG / MAX_ROLL_DEG */
} arm_block_t;

typedef enum {
    DRIVE_BLOCK_NONE = 0,       /* the Pi is driving */
    DRIVE_BLOCK_DISARMED = 1,
    DRIVE_BLOCK_PI = 2,         /* no fresh NAV AUTO or HBT: no route, STOP, or the Pi is silent */
    DRIVE_BLOCK_IMU = 3,        /* IMU stale while armed (ARM_REQUIRES_IMU 0 only) */
    DRIVE_BLOCK_HOLD = 4        /* STOP from the dashboard (App/shore_link.h): motors off until released */
} drive_block_t;

extern volatile heading_debug_t heading_debug;

void Control_Init(controller_t *control);
void Camera_Tick(const pi_command_t *pi, uint32_t now_ms);
void Control_Tick(controller_t *control, const pi_command_t *pi, const bno055_euler_t *imu, uint32_t now_ms);
