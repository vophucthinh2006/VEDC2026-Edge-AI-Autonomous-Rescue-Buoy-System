#include "actuators.h"
#include "app_config.h"
#include "tim.h"
#include <math.h>

/* Order matches servo_id_t. */
static const uint32_t servo_channel[SERVO_COUNT] = {
    TIM_CHANNEL_1,  /* PA6 camera */
    TIM_CHANNEL_2,  /* PA7 front rudder */
    TIM_CHANNEL_4,  /* PB1 right rudder */
    TIM_CHANNEL_3,  /* PB0 left rudder */
};
const uint16_t actuator_servo_center[SERVO_COUNT] = {
    SERVO_CENTER_US, SERVO_CENTER_FRONT_US, SERVO_CENTER_RIGHT_US, SERVO_CENTER_LEFT_US,
};
/* Order matches esc_id_t. TIM2 counts microseconds, like TIM3. */
static const uint32_t esc_channel[ESC_COUNT] = {
    TIM_CHANNEL_2,  /* PA1 front motor */
    TIM_CHANNEL_4,  /* PA3 right motor */
    TIM_CHANNEL_1,  /* PA5 left motor */
};
static const uint16_t esc_stop[ESC_COUNT] = { ESC_FRONT_NEUTRAL_US, ESC_STOP_US, ESC_STOP_US };

volatile actuator_debug_t actuator_debug;

static bool escs_started;
static float rear_out[2];        /* left, right */
static float front_out;
static int8_t front_dir;         /* sign of the last non-zero front output */
static uint32_t front_zero_ms;   /* when the front output last reached zero */
static uint32_t last_apply_ms;

static uint32_t clamp_u32(uint32_t value, uint32_t low, uint32_t high) { return value < low ? low : (value > high ? high : value); }
static float clamp_f(float value, float low, float high) { return value < low ? low : (value > high ? high : value); }
static int8_t sign(float value) { return value > 0.0f ? 1 : (value < 0.0f ? -1 : 0); }

void Actuators_WriteServo(servo_id_t id, uint32_t us) {
    us = id == SERVO_CAMERA ? clamp_u32(us, CAM_SERVO_MIN_US, CAM_SERVO_MAX_US) : clamp_u32(us, SERVO_MIN_US, SERVO_MAX_US);
    __HAL_TIM_SET_COMPARE(&htim3, servo_channel[id], us);
    actuator_debug.servo_us[id] = (uint16_t)us;
}

void Actuators_WriteEsc(esc_id_t id, uint32_t us) {
    if (!escs_started) return;
    __HAL_TIM_SET_COMPARE(&htim2, esc_channel[id], clamp_u32(us, ESC_STOP_US, ESC_MAX_US));
}

void Actuators_Init(void) {
    for (int i = 0; i < SERVO_COUNT; i++) {
        Actuators_WriteServo((servo_id_t)i, actuator_servo_center[i]);
        HAL_TIM_PWM_Start(&htim3, servo_channel[i]);
    }
    for (int i = 0; i < ESC_COUNT; i++) actuator_debug.esc_us[i] = esc_stop[i];
}

void Actuators_StartEscs(void) {
    if (escs_started) return;
    escs_started = true;
    for (int i = 0; i < ESC_COUNT; i++) {
        Actuators_WriteEsc((esc_id_t)i, esc_stop[i]);
        HAL_TIM_PWM_Start(&htim2, esc_channel[i]);
    }
}

bool Actuators_EscsStarted(void) { return escs_started; }

/* Magnitude rises by at most step, falls at once. out and target never
   differ in sign here: rear outputs are never negative and the front one is
   cut to zero before it reverses. */
static float ramp(float out, float target, float step) {
    if (fabsf(target) <= fabsf(out)) return target;
    return target > out ? fminf(target, out + step) : fmaxf(target, out - step);
}

static float front_step(float target, float step, uint32_t now_ms) {
    int8_t want = sign(target);
    if (want != 0 && want == -front_dir) {
        if (front_out != 0.0f) { front_out = 0.0f; front_zero_ms = now_ms; }
        if ((uint32_t)(now_ms - front_zero_ms) < ESC_FRONT_REVERSE_DELAY_MS) return 0.0f;
        front_dir = 0;
    }
    float previous = front_out;
    front_out = ramp(front_out, target, step);
    if (front_out != 0.0f) front_dir = sign(front_out);
    else if (previous != 0.0f) front_zero_ms = now_ms;
    return front_out;
}

static void write_rudder(servo_id_t id, float steer, int dir, float gain) {
    float offset = steer * (float)dir * gain * (float)RUDDER_SPAN_US;
    Actuators_WriteServo(id, (uint32_t)((float)actuator_servo_center[id] + offset));
}

/* Rear demand 0..1 to a pulse: 0 stops, anything above starts where the ESC turns (ESC_REAR_START_US)
   and full demand stays below where it loses sync (ESC_REAR_MAX_US). */
static uint16_t rear_us(float out) {
    if (out <= 0.0f) return (uint16_t)ESC_STOP_US;
    return (uint16_t)((float)ESC_REAR_START_US + clamp_f(out, 0.0f, 1.0f) * (float)(ESC_REAR_MAX_US - ESC_REAR_START_US));
}

static void write_escs(void) {
    uint16_t us[ESC_COUNT];
    us[ESC_FRONT] = (uint16_t)((float)ESC_FRONT_NEUTRAL_US + (float)ESC_FRONT_DIR * front_out * (float)(ESC_MAX_US - ESC_FRONT_NEUTRAL_US));
    us[ESC_LEFT] = rear_us(rear_out[0]);
    us[ESC_RIGHT] = rear_us(rear_out[1]);
    for (int i = 0; i < ESC_COUNT; i++) {
        actuator_debug.esc_us[i] = us[i];
#if ACTUATORS_ENABLED
        Actuators_WriteEsc((esc_id_t)i, us[i]);
#else
        Actuators_WriteEsc((esc_id_t)i, esc_stop[i]);
#endif
    }
}

void Actuators_Apply(const actuator_cmd_t *cmd, uint32_t now_ms) {
    uint32_t elapsed = now_ms - last_apply_ms;
    last_apply_ms = now_ms;
    if (elapsed > 2U * CONTROL_PERIOD_MS) elapsed = CONTROL_PERIOD_MS;   /* first call, or a stalled loop */
    float step = ESC_RAMP_PER_S * (float)elapsed / 1000.0f;

    actuator_debug.cmd = *cmd;
    rear_out[0] = ramp(rear_out[0], clamp_f(cmd->rear_left, 0.0f, 1.0f), step);
    rear_out[1] = ramp(rear_out[1], clamp_f(cmd->rear_right, 0.0f, 1.0f), step);
    front_step(clamp_f(cmd->front, -1.0f, 1.0f), step, now_ms);
    write_escs();

    float steer = clamp_f(cmd->steer, -1.0f, 1.0f);
    write_rudder(SERVO_FRONT, steer, RUDDER_DIR_FRONT, RUDDER_FRONT_GAIN);
    write_rudder(SERVO_RIGHT, steer, RUDDER_DIR_REAR, 1.0f);
    write_rudder(SERVO_LEFT, steer, RUDDER_DIR_REAR, 1.0f);
}

void Actuators_Stop(void) {
    static const actuator_cmd_t zero = { 0.0f, 0.0f, 0.0f, 0.0f };
    actuator_debug.cmd = zero;
    rear_out[0] = rear_out[1] = 0.0f;
    if (front_out != 0.0f) front_zero_ms = HAL_GetTick();
    front_out = 0.0f;
    last_apply_ms = HAL_GetTick();
    write_escs();
    for (int i = SERVO_FRONT; i < SERVO_COUNT; i++) Actuators_WriteServo((servo_id_t)i, actuator_servo_center[i]);
}
