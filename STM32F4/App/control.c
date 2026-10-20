#include "control.h"
#include "app_config.h"
#include "buzzer.h"
#include "camera_pan.h"
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
    ConfirmButton_Init(&control->confirm, HAL_GetTick());
    FrontMotor_Init(&control->front);
    control->operating_mode = 'S';
}

static void heading_off(controller_t *control) {
    HeadingController_Reset(&control->heading);
    control->heading_cmd = 0.0f;
    control->heading_target_valid = false;
    control->heading_state = HEADING_OFF;
    control->operating_mode = 'S';
}

/* A fault (disarmed, overturned, IMU lost): every motor off at once, rudders centred, no braking. */
static void stop_heading(controller_t *control) {
    heading_off(control);
    FrontMotor_Reset(&control->front);
    Actuators_Stop();
}

/* Told to stop (NAV STOP, the Pi silent, STOP from the dashboard): rear motors off, and the front
   motor brakes the boat for a moment (App/front_motor.c) before it rests too. */
static void stop_braking(controller_t *control, uint32_t now_ms) {
    heading_off(control);
    float front = FrontMotor_Command(&control->front, 0.0f, now_ms);
    if (front == 0.0f) { Actuators_Stop(); return; }
    actuator_cmd_t cmd = { 0.0f, 0.0f, front, 0.0f };
    Actuators_Apply(&cmd, now_ms);
}

/* Runs the PID on the locked target and steers cmd with it, around the
   throttle already in cmd->rear_left. */
static void heading_steer(controller_t *control, const bno055_euler_t *imu, actuator_cmd_t *cmd) {
    control->heading_cmd = HeadingController_Update(&control->heading, control->heading_target_deg,
                                                    imu->heading_deg, imu->yaw_rate_dps, DT_S);
    Heading_Mix(control->heading_cmd, cmd->rear_left, &cmd->rear_left, &cmd->rear_right, &cmd->steer);
}

static void publish_debug(const controller_t *control, const bno055_euler_t *imu, bool imu_fresh,
                          const actuator_cmd_t *cmd, drive_block_t block) {
    heading_debug.roll_deg = imu->roll_deg;
    heading_debug.pitch_deg = imu->pitch_deg;
    heading_debug.block = (uint8_t)block;
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

/* Follows the Pi's CAM angle, gliding there (App/camera_pan.c). Independent of arming: the camera
   only looks, and Actuators_Stop leaves it alone. */
void Camera_Tick(const pi_command_t *pi, uint32_t now_ms) {
    static camera_pan_t pan;
    static uint32_t last_ms;
    if (pi->last_cam_ms == 0U) {
        CameraPan_Init(&pan);
        last_ms = now_ms;
        return;
    }
    float dt = (float)(uint32_t)(now_ms - last_ms) / 1000.0f;
    last_ms = now_ms;
    if (dt > 0.1f) dt = 0.1f;   /* a stalled loop: no leap */
    float target = clamp(pi->cam_pan_deg, -CAM_PAN_LIMIT_DEG, CAM_PAN_LIMIT_DEG);
    float angle = CameraPan_Step(&pan, target, dt);
    float us = (float)actuator_servo_center[SERVO_CAMERA] + (float)CAM_PAN_DIR * CAM_PAN_US_PER_DEG * angle;
    Actuators_WriteServo(SERVO_CAMERA, (uint32_t)us);
}

/* There is no RC transmitter: the boat is armed whenever it is safe, and AUTO is the only mode. The
   motors turn only while the Pi drives (a fresh NAV AUTO within its TTL and a fresh HBT). The Pi
   sends STOP with no route from the dashboard; a Pi that goes silent stops them within the NAV TTL
   (300 ms) or HBT_TIMEOUT_MS. */
void Control_Tick(controller_t *control, const pi_command_t *pi, const bno055_euler_t *imu, uint32_t now_ms) {
    /* Fresh = a good read within IMU_TIMEOUT_MS. A single failed I2C read only
       clears imu->valid for one tick and must not disarm the boat; the angles
       in imu are still the last good ones. */
    bool imu_fresh = control->last_imu_ms != 0U && fresh(now_ms, control->last_imu_ms, IMU_TIMEOUT_MS);
    if (!imu->valid && heading_debug.imu_fail < 0xFFFFU) heading_debug.imu_fail++;
    /* PE4 is the victim's confirm button, not an E-stop: it is counted for the Pi and stops nothing. */
    control->estop = false;
    ConfirmButton_Update(&control->confirm, HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_RESET, now_ms);
    control->overturned = imu_fresh && (fabsf(imu->pitch_deg) > MAX_PITCH_DEG || fabsf(imu->roll_deg) > MAX_ROLL_DEG);

    bool was_armed = control->armed;
    control->armed = (imu_fresh || !ARM_REQUIRES_IMU || BENCH_IGNORE_IMU) && !control->overturned;
    arm_block_t arm_block = control->armed ? ARM_BLOCK_NONE : control->overturned ? ARM_BLOCK_OVERTURNED : ARM_BLOCK_IMU;
    if (control->armed != was_armed) Buzzer_Beep(control->armed ? 1U : 2U);
    if (was_armed && !control->armed) heading_debug.last_disarm = (uint8_t)arm_block;
    heading_debug.arm_block = (uint8_t)arm_block;
    if (!control->armed) { stop_heading(control); publish_debug(control, imu, imu_fresh, NULL, DRIVE_BLOCK_DISARMED); return; }

    /* STOP from the dashboard wins over the Pi: the boat stays where it is, the route is kept. */
    if (control->hold) { stop_braking(control, now_ms); publish_debug(control, imu, imu_fresh, NULL, DRIVE_BLOCK_HOLD); return; }
    bool nav_fresh = pi->mode == NAV_AUTO && pi->ttl_ms > 0U && fresh(now_ms, pi->last_nav_ms, pi->ttl_ms);
    bool hbt_fresh = pi->pi_link_ok && fresh(now_ms, pi->last_hbt_ms, HBT_TIMEOUT_MS);
    if (!nav_fresh || !hbt_fresh) { stop_braking(control, now_ms); publish_debug(control, imu, imu_fresh, NULL, DRIVE_BLOCK_PI); return; }
    /* A stale heading must never drive a motor on the water. Only BENCH_IGNORE_IMU, for the boat
       on its stand, lets it run on the held one. */
    if (!imu_fresh && !BENCH_IGNORE_IMU) { stop_heading(control); publish_debug(control, imu, imu_fresh, NULL, DRIVE_BLOCK_IMU); return; }
    bno055_euler_t held = *imu;
    if (!imu_fresh) held.yaw_rate_dps = 0.0f;     /* the last rate read would brake for ever */
    imu = &held;
    if (control->heading_state != HEADING_AUTO) HeadingController_Reset(&control->heading);
    control->heading_state = HEADING_AUTO;
    control->heading_target_deg = Heading_Wrap360(pi->heading_deg);
    control->heading_target_valid = true;
    control->operating_mode = 'A';
    /* Pi speed in m/s is deliberately mapped conservatively; tune only on water after dry tests. */
    float throttle = clamp(pi->speed_mps / 1.0f, 0.0f, 0.50f);
    /* The front motor pushes with the rear ones, or brakes when the Pi has just slowed the boat. */
    actuator_cmd_t cmd = { throttle, throttle, FrontMotor_Command(&control->front, throttle, now_ms), 0.0f };
    if (throttle <= 0.0f) {
        /* Speed 0 (e.g. holding standoff from a person): no pivoting next to
           the victim, motors rest and the rudders centre. */
        HeadingController_Reset(&control->heading);
        control->heading_cmd = 0.0f;
    } else {
        heading_steer(control, imu, &cmd);
    }
    Actuators_Apply(&cmd, now_ms);
    publish_debug(control, imu, imu_fresh, &cmd, DRIVE_BLOCK_NONE);
}
