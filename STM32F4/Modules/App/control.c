#include "control.h"
#include "app_config.h"
#include <math.h>
#include <string.h>

static float clamp(float value, float low, float high) { return value < low ? low : (value > high ? high : value); }
static float heading_error(float target, float current) { float e = fmodf(target - current + 540.0f, 360.0f) - 180.0f; return e; }

void Control_Init(controller_t *control) { memset(control, 0, sizeof(*control)); }

void Control_Tick(controller_t *control, const pi_command_t *pi, const ibus_state_t *rc, const bno055_euler_t *imu, const actuator_driver_t *actuators, uint32_t now_ms) {
    bool rc_fresh = rc->valid && (uint32_t)(now_ms - rc->last_rx_ms) <= RC_TIMEOUT_MS;
    bool imu_fresh = imu->valid && (uint32_t)(now_ms - control->last_imu_ms) <= IMU_TIMEOUT_MS;
    /* Normally-closed contact to GND against an internal pull-up: released
       reads low, pressed reads high, and a cut wire also reads high. */
    control->estop = HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_SET;
    control->overturned = imu->valid && (fabsf(imu->pitch_deg) > MAX_PITCH_DEG || fabsf(imu->roll_deg) > MAX_ROLL_DEG);
    bool auto_requested = rc_fresh && rc->channel[RC_CH_MODE] >= RC_MODE_AUTO_THRESHOLD;
    bool arm_requested = rc_fresh && rc->channel[RC_CH_ARM] >= RC_ARM_THRESHOLD && rc->channel[RC_CH_THROTTLE] <= 1050U;
    if (!arm_requested || control->estop || control->overturned || !imu_fresh) control->armed = false;
    else if (!control->armed && arm_requested) control->armed = true;
    if (!control->armed || control->estop || control->overturned || !rc_fresh) { Actuators_Stop(actuators); return; }
    if (!auto_requested) { Actuators_SetManual(actuators, rc->channel[RC_CH_THROTTLE], rc->channel[RC_CH_YAW]); return; }
    bool nav_fresh = pi->mode == NAV_AUTO && pi->ttl_ms > 0U && (uint32_t)(now_ms - pi->last_nav_ms) <= pi->ttl_ms;
    bool hbt_fresh = pi->pi_link_ok && (uint32_t)(now_ms - pi->last_hbt_ms) <= HBT_TIMEOUT_MS;
    if (!nav_fresh || !hbt_fresh) { Actuators_Stop(actuators); return; }
    float error = heading_error(pi->heading_deg, imu->heading_deg);
    control->heading_integral = clamp(control->heading_integral + error * 0.01f, -40.0f, 40.0f);
    float yaw = clamp(0.012f * error + 0.002f * control->heading_integral, -0.35f, 0.35f);
    /* Pi speed in m/s is deliberately mapped conservatively; tune only on water after dry tests. */
    float throttle = clamp(pi->speed_mps / 1.0f, 0.0f, 0.50f);
    Actuators_SetAuto(actuators, throttle, yaw);
}
