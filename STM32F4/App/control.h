#pragma once

#include "actuators.h"
#include "imu.h"
#include "gps_nmea.h"
#include "ibus.h"
#include "uart_protocol.h"
#include "heading_controller.h"
#include <stdbool.h>
#include <stdint.h>

/* Who is steering by heading right now. */
typedef enum {
    HEADING_OFF = 0,
    HEADING_HOLD_WAIT = 1,   /* RC hold requested, waiting for the turn to settle */
    HEADING_HOLD = 2,        /* RC hold locked on heading_target_deg */
    HEADING_AUTO = 3         /* target from the Pi NAV packet */
} heading_state_t;

typedef struct {
    bool armed;
    bool arm_switch_released;   /* arm switch seen off since the last arming */
    bool manual_ready;          /* speed stick centred since entering MANUAL */
    bool overturned;
    bool motor_fault;
    bool estop;
    bool auto_requested;        /* RC mode switch at AUTO, whether or not the Pi is steering yet */
    heading_controller_t heading;
    float heading_target_deg;
    float heading_cmd;            /* PID output u, -1..1, positive turns the bow right */
    bool heading_target_valid;
    uint8_t heading_state;        /* heading_state_t */
    uint32_t hold_wait_ms;        /* when HEADING_HOLD_WAIT started */
    char operating_mode;          /* 'S' stopped, 'M' manual, 'A' AUTO heading hold */
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
    uint16_t ch_steer;     /* raw iBUS CH1, CH2, CH3, CH5, CH6 */
    uint16_t ch_speed;
    uint16_t ch_power;
    uint16_t ch_mode;
    uint16_t ch_arm;
    uint16_t imu_fail;     /* control ticks with a failed IMU read since boot */
    uint8_t state;         /* heading_state_t */
    uint8_t imu_ok;
    uint8_t block;         /* hold_block_t: why the RC hold is not engaged */
    uint8_t armed;
    uint8_t arm_block;     /* arm_block_t: why the boat is not armed */
    uint8_t estop;
    uint8_t last_disarm;   /* arm_block_t that ended the last arming, 0 = none yet */
} heading_debug_t;

typedef enum {
    ARM_BLOCK_NONE = 0,
    ARM_BLOCK_RC_LOST = 1,
    ARM_BLOCK_IMU = 2,
    ARM_BLOCK_ESTOP = 3,
    ARM_BLOCK_OVERTURNED = 4,
    ARM_BLOCK_SWITCH_OFF = 5,       /* CH6 below RC_ARM_THRESHOLD */
    ARM_BLOCK_SWITCH_NOT_CYCLED = 6,/* SwD must be seen OFF before each arming */
    ARM_BLOCK_MODE_AUTO = 7,        /* CH5 at AUTO */
    ARM_BLOCK_POWER = 8,            /* CH3 above RC_POWER_MIN_US */
    ARM_BLOCK_SPEED = 9             /* CH2 not centred */
} arm_block_t;

typedef enum {
    HOLD_BLOCK_NONE = 0,
    HOLD_BLOCK_DISARMED = 1,
    HOLD_BLOCK_AUTO = 2,          /* SwB at AUTO: the Pi steers */
    HOLD_BLOCK_SPEED_RESET = 3,   /* speed stick not yet centred after arming or AUTO */
    HOLD_BLOCK_IMU = 4,           /* IMU (Euler or gyro) stale */
    HOLD_BLOCK_THROTTLE = 5,      /* no forward rear throttle: CH2 forward and CH3 up */
    HOLD_BLOCK_STEERING = 6,      /* CH1 outside RC_HOLD_STEER_BAND_US of 1500 */
    HOLD_BLOCK_DISABLED = 7       /* RC_HEADING_HOLD_ENABLED is 0 */
} hold_block_t;

extern volatile heading_debug_t heading_debug;

void Control_Init(controller_t *control);
void Camera_Tick(const pi_command_t *pi);
void Control_Tick(controller_t *control, const pi_command_t *pi, const ibus_state_t *rc, const bno055_euler_t *imu, uint32_t now_ms);
