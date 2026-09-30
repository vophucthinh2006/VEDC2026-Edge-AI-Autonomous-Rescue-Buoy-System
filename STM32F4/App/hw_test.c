#include "hw_test.h"
#include "actuators.h"
#include "app_config.h"
#include "tim.h"
#include <stdbool.h>

#define BUZZER_ON_PULSE  125U   /* half of TIM8 ARR 249: 4 kHz square wave */

/* hw_test_t orders servos and ESCs like servo_id_t and esc_id_t. */
static const uint16_t esc_stop[ESC_COUNT] = { ESC_FRONT_NEUTRAL_US, ESC_STOP_US, ESC_STOP_US };

volatile hw_test_t hw_test;
static uint32_t last_ms;

static uint32_t clamp_u32(uint32_t value, uint32_t low, uint32_t high) { return value < low ? low : (value > high ? high : value); }
static uint32_t count_down(uint32_t value, uint32_t elapsed) { return value > elapsed ? value - elapsed : 0U; }

/* Bench limits per ESC: rear ones throttle up from stop, the front one swings
   both ways around neutral. */
static uint32_t limit_esc(int i, uint32_t us) {
    if (i == ESC_FRONT) return clamp_u32(us, ESC_FRONT_NEUTRAL_US - HW_TEST_ESC_FRONT_SPAN_US, ESC_FRONT_NEUTRAL_US + HW_TEST_ESC_FRONT_SPAN_US);
    return clamp_u32(us, ESC_STOP_US, HW_TEST_ESC_MAX_US);
}

void HwTest_Init(void) {
    Actuators_Init();
    for (int i = 0; i < SERVO_COUNT; i++) hw_test.servo_us[i] = actuator_servo_center[i];
    for (int i = 0; i < ESC_COUNT; i++) hw_test.esc_us[i] = esc_stop[i];
    hw_test.servo_run_ms = 0U;
    hw_test.esc_run_ms = 0U;
    hw_test.esc_enable = 0U;
#if HW_TEST_ESC_ARM_AT_BOOT
    Actuators_StartEscs();
    hw_test.esc_enable = 1U;
#endif
    hw_test.sweep_half_ms = 0U;
    hw_test.buzzer_ms = 200U;   /* one beep: firmware is up */
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0U);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    last_ms = HAL_GetTick();
}

void HwTest_Tick(uint32_t now_ms) {
    uint32_t elapsed = now_ms - last_ms;
    last_ms = now_ms;

    bool servo_run = hw_test.servo_run_ms > 0U;
    uint32_t half = hw_test.sweep_half_ms;
    bool sweep_right = half > 0U && (now_ms / half) % 2U == 0U;   /* lower pulse turns right */
    for (int i = 0; i < SERVO_COUNT; i++) {
        uint32_t us = servo_run ? hw_test.servo_us[i] : actuator_servo_center[i];
        if (half > 0U && i != SERVO_CAMERA) {
            int32_t offset = sweep_right ? -(int32_t)HW_TEST_SWEEP_US : (int32_t)HW_TEST_SWEEP_US;
            us = (uint32_t)((int32_t)actuator_servo_center[i] + offset);
        }
        Actuators_WriteServo((servo_id_t)i, us);
    }
    hw_test.servo_run_ms = count_down(hw_test.servo_run_ms, elapsed);

    if (hw_test.esc_enable == 1U) Actuators_StartEscs();
    if (hw_test.esc_run_ms > HW_TEST_ESC_MAX_RUN_MS) hw_test.esc_run_ms = HW_TEST_ESC_MAX_RUN_MS;
    bool esc_run = Actuators_EscsStarted() && hw_test.esc_run_ms > 0U;
    for (int i = 0; i < ESC_COUNT; i++)
        Actuators_WriteEsc((esc_id_t)i, esc_run ? limit_esc(i, hw_test.esc_us[i]) : esc_stop[i]);
    hw_test.esc_run_ms = count_down(hw_test.esc_run_ms, elapsed);

    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, hw_test.buzzer_ms > 0U ? BUZZER_ON_PULSE : 0U);
    hw_test.buzzer_ms = (uint16_t)count_down(hw_test.buzzer_ms, elapsed);
}
