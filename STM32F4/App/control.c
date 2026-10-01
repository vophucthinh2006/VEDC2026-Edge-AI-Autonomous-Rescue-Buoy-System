#include "control.h"
#include "app_config.h"
#include "buzzer.h"
#include "manual.h"
#include <math.h>
#include <string.h>

#define DT_S (CONTROL_PERIOD_MS / 1000.0f)

volatile heading_debug_t heading_debug;

static float clamp(float value, float low, float high) { return value < low ? low : (value > high ? high : value); }

/* Received no more than timeout_ms before now_ms. UART interrupts stamp their
   data with HAL_GetTick() while the loop still works with the now_ms it read
   before the IMU transfer, so a stamp can be a tick or two NEWER than now_ms:
   unsigned now - stamp would then wrap to ~4e9 and read as stale. */
static bool fresh(uint32_t now_ms, uint32_t stamp_ms, uint32_t timeout_ms) {
    return (int32_t)(now_ms - stamp_ms) <= (int32_t)timeout_ms;
}

void Control_Init(controller_t *control) {
    memset(control, 0, sizeof(*control));
    HeadingController_Init(&control->heading);
    control->operating_mode = 'S';
}

/* Heading loop off: PID cleared, no target shown. */
static void heading_off(controller_t *control) {
    HeadingController_Reset(&control->heading);
    control->heading_cmd = 0.0f;
    control->heading_target_valid = false;
    control->heading_state = HEADING_OFF;
}

static void stop_heading(controller_t *control) {
    heading_off(control);
    control->operating_mode = 'S';
    Actuators_Stop();
}

/* Runs the PID on the locked target and steers cmd with it, around the
   throttle already in cmd->rear_left. */
static void heading_steer(controller_t *control, const bno055_euler_t *imu, actuator_cmd_t *cmd) {
    control->heading_cmd = HeadingController_Update(&control->heading, control->heading_target_deg,
                                                    imu->heading_deg, imu->yaw_rate_dps, DT_S);
    Heading_Mix(control->heading_cmd, cmd->rear_left, &cmd->rear_left, &cmd->rear_right, &cmd->steer);
}

#if RC_HEADING_HOLD_ENABLED
/* MANUAL heading hold: forward throttle with the steering centred locks the
   heading once the turn has settled; anything else hands the rudders back. */
static uint8_t rc_heading_hold(controller_t *control, const ibus_state_t *rc, const bno055_euler_t *imu,
                               bool imu_fresh, actuator_cmd_t *cmd, uint32_t now_ms) {
    int32_t steer_offset = (int32_t)rc->channel[RC_CH_STEER] - 1500;
    uint8_t block = !imu_fresh ? HOLD_BLOCK_IMU
                  : cmd->rear_left <= 0.0f ? HOLD_BLOCK_THROTTLE
                  : (steer_offset > (int32_t)RC_HOLD_STEER_BAND_US || steer_offset < -(int32_t)RC_HOLD_STEER_BAND_US) ? HOLD_BLOCK_STEERING
                  : HOLD_BLOCK_NONE;
    if (block != HOLD_BLOCK_NONE) { heading_off(control); return block; }
    if (control->heading_state == HEADING_OFF || control->heading_state == HEADING_AUTO) {
        HeadingController_Reset(&control->heading);
        control->heading_state = HEADING_HOLD_WAIT;
        control->hold_wait_ms = now_ms;
    }
    if (control->heading_state == HEADING_HOLD_WAIT) {
        /* Locking mid-turn would make the boat swing back past where the
           driver let go; wait for it to settle, but not forever. */
        if (fabsf(imu->yaw_rate_dps) > RC_HOLD_LOCK_RATE_DPS &&
            (uint32_t)(now_ms - control->hold_wait_ms) < RC_HOLD_LOCK_TIMEOUT_MS) return HOLD_BLOCK_NONE;
        control->heading_target_deg = imu->heading_deg;
        control->heading_target_valid = true;
        control->heading_state = HEADING_HOLD;
    }
    heading_steer(control, imu, cmd);
    return HOLD_BLOCK_NONE;
}
#endif

static void publish_debug(const controller_t *control, const ibus_state_t *rc, const bno055_euler_t *imu,
                          bool imu_fresh, const actuator_cmd_t *cmd, uint8_t block) {
    heading_debug.ch_steer = rc->channel[RC_CH_STEER];
    heading_debug.ch_speed = rc->channel[RC_CH_SPEED];
    heading_debug.ch_power = rc->channel[RC_CH_POWER];
    heading_debug.ch_mode = rc->channel[RC_CH_MODE];
    heading_debug.ch_arm = rc->channel[RC_CH_ARM];
    heading_debug.roll_deg = imu->roll_deg;
    heading_debug.pitch_deg = imu->pitch_deg;
    heading_debug.block = block;
    heading_debug.armed = control->armed ? 1U : 0U;
    heading_debug.target_deg = control->heading_target_valid ? control->heading_target_deg : -1.0f;
    heading_debug.yaw_deg = imu->heading_deg;
    heading_debug.yaw_rate_dps = imu->yaw_rate_dps;
    heading_debug.error_deg = control->heading.error_deg;
    heading_debug.p = control->heading.p;
    heading_debug.i = control->heading.i;
    heading_debug.d = control->heading.d;
    heading_debug.u = control->heading_cmd;
    heading_debug.steer = cmd != NULL ? cmd->steer : 0.0f;
    heading_debug.rear_left = cmd != NULL ? cmd->rear_left : 0.0f;
    heading_debug.rear_right = cmd != NULL ? cmd->rear_right : 0.0f;
    heading_debug.state = control->heading_state;
    heading_debug.imu_ok = imu_fresh ? 1U : 0U;
}

/* Follows the Pi's CAM angle. Independent of arming: the camera only looks, and Actuators_Stop leaves it alone. */
void Camera_Tick(const pi_command_t *pi) {
    if (pi->last_cam_ms == 0U) return;
    float pan = clamp(pi->cam_pan_deg, -CAM_PAN_LIMIT_DEG, CAM_PAN_LIMIT_DEG);
    float us = (float)actuator_servo_center[SERVO_CAMERA] + (float)CAM_PAN_DIR * CAM_PAN_US_PER_DEG * pan;
    Actuators_WriteServo(SERVO_CAMERA, (uint32_t)us);
}

void Control_Tick(controller_t *control, const pi_command_t *pi, const ibus_state_t *rc, const bno055_euler_t *imu, uint32_t now_ms) {
    bool rc_fresh = rc->valid && fresh(now_ms, rc->last_rx_ms, RC_TIMEOUT_MS);
    /* Fresh = a good read within IMU_TIMEOUT_MS. A single failed I2C read only
       clears imu->valid for one tick and must not disarm the boat; the angles
       in imu are still the last good ones. */
    bool imu_fresh = control->last_imu_ms != 0U && fresh(now_ms, control->last_imu_ms, IMU_TIMEOUT_MS);
    if (!imu->valid && heading_debug.imu_fail < 0xFFFFU) heading_debug.imu_fail++;
    /* Contact to GND against an internal pull-up. Normally closed: released
       reads low, pressed and a cut wire read high. Normally open reads the
       other way round and cannot tell released from a cut wire. */
    control->estop = HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == (ESTOP_CONTACT_NC ? GPIO_PIN_SET : GPIO_PIN_RESET);
    control->overturned = imu_fresh && (fabsf(imu->pitch_deg) > MAX_PITCH_DEG || fabsf(imu->roll_deg) > MAX_ROLL_DEG);
    bool auto_requested = rc_fresh && rc->channel[RC_CH_MODE] >= RC_MODE_AUTO_THRESHOLD;
    bool was_armed = control->armed;
    bool arm_switch = rc_fresh && rc->channel[RC_CH_ARM] >= RC_ARM_THRESHOLD;
    bool safe = rc_fresh && (imu_fresh || !ARM_REQUIRES_IMU) && !control->estop && !control->overturned;
    /* Each arming needs the switch seen off first, so a link that comes back,
       a cleared E-stop or a reset never re-arms with the switch left on. */
    if (rc_fresh && !arm_switch) control->arm_switch_released = true;
    if (!arm_switch || !safe) control->armed = false;
    else if (!control->armed && control->arm_switch_released && !auto_requested &&
             rc->channel[RC_CH_POWER] <= RC_POWER_MIN_US && Manual_SpeedCentred(rc)) {
        control->armed = true;
        control->arm_switch_released = false;
        control->manual_ready = true;
    }
    if (control->armed != was_armed) Buzzer_Beep(control->armed ? 1U : 2U);
    if (was_armed && !control->armed) {
        heading_debug.last_disarm = !rc_fresh ? ARM_BLOCK_RC_LOST
            : (!imu_fresh && ARM_REQUIRES_IMU) ? ARM_BLOCK_IMU
            : control->estop ? ARM_BLOCK_ESTOP
            : control->overturned ? ARM_BLOCK_OVERTURNED
            : ARM_BLOCK_SWITCH_OFF;
    }
    heading_debug.arm_block = control->armed ? ARM_BLOCK_NONE
        : !rc_fresh ? ARM_BLOCK_RC_LOST
        : (!imu_fresh && ARM_REQUIRES_IMU) ? ARM_BLOCK_IMU
        : control->estop ? ARM_BLOCK_ESTOP
        : control->overturned ? ARM_BLOCK_OVERTURNED
        : !arm_switch ? ARM_BLOCK_SWITCH_OFF
        : !control->arm_switch_released ? ARM_BLOCK_SWITCH_NOT_CYCLED
        : auto_requested ? ARM_BLOCK_MODE_AUTO
        : rc->channel[RC_CH_POWER] > RC_POWER_MIN_US ? ARM_BLOCK_POWER
        : ARM_BLOCK_SPEED;
    heading_debug.estop = control->estop ? 1U : 0U;
    if (!control->armed) { stop_heading(control); publish_debug(control, rc, imu, imu_fresh, NULL, HOLD_BLOCK_DISARMED); return; }
    actuator_cmd_t cmd;
    if (!auto_requested) {
        control->operating_mode = 'M';
        /* Back from AUTO, wait for the speed stick to be centred once, or a
           stick held forward would launch the boat on the switch flip. */
        if (!control->manual_ready) control->manual_ready = Manual_SpeedCentred(rc);
        if (!control->manual_ready) {
            heading_off(control);
            Actuators_Stop();
            publish_debug(control, rc, imu, imu_fresh, NULL, HOLD_BLOCK_SPEED_RESET);
            return;
        }
        Manual_Mix(rc, &cmd);
#if RC_HEADING_HOLD_ENABLED
        uint8_t block = rc_heading_hold(control, rc, imu, imu_fresh, &cmd, now_ms);
#else
        uint8_t block = HOLD_BLOCK_DISABLED;
        heading_off(control);
#endif
        Actuators_Apply(&cmd, now_ms);
        publish_debug(control, rc, imu, imu_fresh, &cmd, block);
        return;
    }
    control->manual_ready = false;
    bool nav_fresh = pi->mode == NAV_AUTO && pi->ttl_ms > 0U && fresh(now_ms, pi->last_nav_ms, pi->ttl_ms);
    bool hbt_fresh = pi->pi_link_ok && fresh(now_ms, pi->last_hbt_ms, HBT_TIMEOUT_MS);
    /* AUTO always requires a fresh IMU, even when the bench-only arming
       override is enabled. A stale heading must never drive a motor. */
    if (!imu_fresh || !nav_fresh || !hbt_fresh) { stop_heading(control); publish_debug(control, rc, imu, imu_fresh, NULL, HOLD_BLOCK_AUTO); return; }
    if (control->heading_state != HEADING_AUTO) HeadingController_Reset(&control->heading);
    control->heading_state = HEADING_AUTO;
    control->heading_target_deg = Heading_Wrap360(pi->heading_deg);
    control->heading_target_valid = true;
    control->operating_mode = 'A';
    /* Pi speed in m/s is deliberately mapped conservatively; tune only on water after dry tests. */
    float throttle = clamp(pi->speed_mps / 1.0f, 0.0f, 0.50f);
    cmd = (actuator_cmd_t){ throttle, throttle, 0.0f, 0.0f };
    if (throttle <= 0.0f) {
        /* Speed 0 (e.g. holding standoff from a person): no pivoting next to
           the victim, motors rest and the rudders centre. */
        HeadingController_Reset(&control->heading);
        control->heading_cmd = 0.0f;
    } else {
        heading_steer(control, imu, &cmd);
    }
    Actuators_Apply(&cmd, now_ms);
    publish_debug(control, rc, imu, imu_fresh, &cmd, HOLD_BLOCK_AUTO);
}
