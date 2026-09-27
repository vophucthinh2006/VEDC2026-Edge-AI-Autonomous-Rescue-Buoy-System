#include "control.h"
#include "app_config.h"
#include "buzzer.h"
#include "manual.h"
#include <math.h>
#include <string.h>

static float clamp(float value, float low, float high) { return value < low ? low : (value > high ? high : value); }
static float heading_error(float target, float current) { float e = fmodf(target - current + 540.0f, 360.0f) - 180.0f; return e; }

void Control_Init(controller_t *control) { memset(control, 0, sizeof(*control)); }

void Control_Tick(controller_t *control, const pi_command_t *pi, const ibus_state_t *rc, const bno055_euler_t *imu, uint32_t now_ms) {
    bool rc_fresh = rc->valid && (uint32_t)(now_ms - rc->last_rx_ms) <= RC_TIMEOUT_MS;
    bool imu_fresh = imu->valid && (uint32_t)(now_ms - control->last_imu_ms) <= IMU_TIMEOUT_MS;
    /* Normally-closed contact to GND against an internal pull-up: released
       reads low, pressed reads high, and a cut wire also reads high. */
    control->estop = HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_SET;
    control->overturned = imu->valid && (fabsf(imu->pitch_deg) > MAX_PITCH_DEG || fabsf(imu->roll_deg) > MAX_ROLL_DEG);
    bool auto_requested = rc_fresh && rc->channel[RC_CH_MODE] >= RC_MODE_AUTO_THRESHOLD;
    bool was_armed = control->armed;
    bool arm_switch = rc_fresh && rc->channel[RC_CH_ARM] >= RC_ARM_THRESHOLD;
    bool safe = rc_fresh && (imu_fresh || !ARM_REQUIRES_IMU) && (!control->estop || !ESTOP_ENABLED) && !control->overturned;
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
    if (!control->armed) { Actuators_Stop(); return; }
    if (!auto_requested) {
        /* Back from AUTO, wait for the speed stick to be centred once, or a
           stick held forward would launch the boat on the switch flip. */
        if (!control->manual_ready) control->manual_ready = Manual_SpeedCentred(rc);
        if (!control->manual_ready) { Actuators_Stop(); return; }
        actuator_cmd_t cmd;
        Manual_Mix(rc, &cmd);
        Actuators_Apply(&cmd, now_ms);
        return;
    }
    control->manual_ready = false;
    bool nav_fresh =pi->mode == NAV_AUTO && pi->ttl_ms > 0U && (uint32_t)(now_ms - pi->last_nav_ms) <= pi->ttl_ms;
    bool hbt_fresh = pi->pi_link_ok && (uint32_t)(now_ms - pi->last_hbt_ms) <= HBT_TIMEOUT_MS;
    if (!nav_fresh || !hbt_fresh) { Actuators_Stop(); return; }
    float error = heading_error(pi->heading_deg, imu->heading_deg);
    control->heading_integral = clamp(control->heading_integral + error * 0.01f, -40.0f, 40.0f);
    float yaw = clamp(0.012f * error + 0.002f * control->heading_integral, -0.35f, 0.35f);
    /* Pi speed in m/s is deliberately mapped conservatively; tune only on water after dry tests. */
    float throttle = clamp(pi->speed_mps / 1.0f, 0.0f, 0.50f);
    /* Heading hold still trims with the rear motors only; the rudders stay centred. */
    actuator_cmd_t cmd = { throttle - yaw, throttle + yaw, 0.0f, 0.0f };
    Actuators_Apply(&cmd, now_ms);
}
