#include "actuators.h"
#include "app_config.h"
#include "dshot.h"
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
/* Rear ESCs are a CCR slot of the TIM2 DShot burst (0 = CCR1). */
static const uint32_t esc_dshot_ccr[ESC_COUNT] = {
    0xFFU,  /* PA1 front motor: PWM on TIM5_CH2 */
    3U,     /* PA3 right motor, TIM2_CH4 */
    0U,     /* PA5 left motor, TIM2_CH1 */
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

/* 1000 us or less is DShot stop, 1000..2000 us spans DShot 48..2047. */
static uint16_t dshot_from_us(uint32_t us) {
    if (us <= ESC_STOP_US) return DSHOT_STOP;
    if (us > ESC_MAX_US) us = ESC_MAX_US;
    return (uint16_t)(DSHOT_MIN + (us - ESC_STOP_US) * (DSHOT_MAX - DSHOT_MIN) / (ESC_MAX_US - ESC_STOP_US));
}

void Actuators_WriteServo(servo_id_t id, uint32_t us) {
    us = clamp_u32(us, SERVO_MIN_US, SERVO_MAX_US);
    __HAL_TIM_SET_COMPARE(&htim3, servo_channel[id], us);
    actuator_debug.servo_us[id] = (uint16_t)us;
}

void Actuators_WriteEsc(esc_id_t id, uint32_t us) {
    if (!escs_started) return;
    if (esc_dshot_ccr[id] <= 3U) DShot_Set(esc_dshot_ccr[id], dshot_from_us(us));
    else __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, clamp_u32(us, ESC_STOP_US, ESC_MAX_US));
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
    DShot_Init(&htim2);
    escs_started = true;
    for (int i = 0; i < ESC_COUNT; i++) Actuators_WriteEsc((esc_id_t)i, esc_stop[i]);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
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

static void write_escs(void) {
    uint16_t us[ESC_COUNT];
    us[ESC_FRONT] = (uint16_t)((float)ESC_FRONT_NEUTRAL_US + front_out * (float)(ESC_MAX_US - ESC_FRONT_NEUTRAL_US));
    us[ESC_LEFT] = (uint16_t)((float)ESC_STOP_US + rear_out[0] * (float)(ESC_MAX_US - ESC_STOP_US));
    us[ESC_RIGHT] = (uint16_t)((float)ESC_STOP_US + rear_out[1] * (float)(ESC_MAX_US - ESC_STOP_US));
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
    if (elapsed > 100U) elapsed = 100U;   /* first call after a stop */
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
