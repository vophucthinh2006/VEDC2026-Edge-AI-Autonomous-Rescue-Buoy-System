#include "hw_test.h"
#include "app_config.h"
#include "dshot.h"
#include "tim.h"
#include <stdbool.h>

#define BUZZER_ON_PULSE  125U   /* half of TIM8 ARR 249: 4 kHz square wave */

/* Order matches hw_test_t. Pin map from the Pinout (Hy) sheet. */
static const uint32_t servo_channel[4] = {
    TIM_CHANNEL_1,  /* PA6 camera */
    TIM_CHANNEL_2,  /* PA7 front rudder */
    TIM_CHANNEL_4,  /* PB1 right rudder */
    TIM_CHANNEL_3,  /* PB0 left rudder */
};
static const uint16_t servo_center[4] = {
    SERVO_CENTER_US, SERVO_CENTER_FRONT_US, SERVO_CENTER_RIGHT_US, SERVO_CENTER_LEFT_US,
};
/* Rear ESCs speak DShot300 on TIM2; index is the CCR slot (0 = CCR1).
   The front ESC (PA1) is bidirectional PWM on TIM5_CH2. */
static const uint32_t esc_dshot_ccr[3] = {
    0xFFU,  /* PA1 front motor: PWM, see set_esc() */
    3U,     /* PA3 right motor, TIM2_CH4 */
    0U,     /* PA5 left motor, TIM2_CH1 */
};
static const uint16_t esc_stop[3] = { ESC_FRONT_NEUTRAL_US, ESC_STOP_US, ESC_STOP_US };

volatile hw_test_t hw_test;
static uint32_t last_ms;
static bool esc_started;

static uint32_t clamp_u32(uint32_t value, uint32_t low, uint32_t high) { return value < low ? low : (value > high ? high : value); }
static uint32_t count_down(uint32_t value, uint32_t elapsed) { return value > elapsed ? value - elapsed : 0U; }

/* Keep the microsecond interface of hw_test_t: 1000 us or less is DShot stop,
   1000..2000 us spans DShot 48..2047. */
static uint16_t dshot_from_us(uint32_t us) {
    if (us <= ESC_STOP_US) return DSHOT_STOP;
    if (us > ESC_MAX_US) us = ESC_MAX_US;
    return (uint16_t)(DSHOT_MIN + (us - ESC_STOP_US) * (DSHOT_MAX - DSHOT_MIN) / (ESC_MAX_US - ESC_STOP_US));
}

static void set_esc(int i, uint32_t us) {
    if (esc_dshot_ccr[i] <= 3U) DShot_Set(esc_dshot_ccr[i], dshot_from_us(us));
    else __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, us);
}

/* Bench limits per ESC: rear ones throttle up from stop, the front one swings
   both ways around neutral. */
static uint32_t limit_esc(int i, uint32_t us) {
    if (i == 0) return clamp_u32(us, ESC_FRONT_NEUTRAL_US - HW_TEST_ESC_FRONT_SPAN_US, ESC_FRONT_NEUTRAL_US + HW_TEST_ESC_FRONT_SPAN_US);
    return clamp_u32(us, ESC_STOP_US, HW_TEST_ESC_MAX_US);
}

static void start_escs(void) {
    DShot_Init(&htim2);
    for (int i = 0; i < 3; i++) set_esc(i, esc_stop[i]);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    esc_started = true;
}

void HwTest_Init(void) {
    for (int i = 0; i < 4; i++) {
        hw_test.servo_us[i] = servo_center[i];
        __HAL_TIM_SET_COMPARE(&htim3, servo_channel[i], servo_center[i]);
        HAL_TIM_PWM_Start(&htim3, servo_channel[i]);
    }
    for (int i = 0; i < 3; i++) hw_test.esc_us[i] = esc_stop[i];
    hw_test.servo_run_ms = 0U;
    hw_test.esc_run_ms = 0U;
    hw_test.esc_enable = 0U;
#if HW_TEST_ESC_ARM_AT_BOOT
    start_escs();
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
    for (int i = 0; i < 4; i++) {
        uint32_t us = servo_run ? clamp_u32(hw_test.servo_us[i], SERVO_MIN_US, SERVO_MAX_US) : servo_center[i];
        if (half > 0U && i > 0) {
            int32_t offset = sweep_right ? -(int32_t)HW_TEST_SWEEP_US : (int32_t)HW_TEST_SWEEP_US;
            us = clamp_u32((uint32_t)((int32_t)servo_center[i] + offset), SERVO_MIN_US, SERVO_MAX_US);
        }
        __HAL_TIM_SET_COMPARE(&htim3, servo_channel[i], us);
    }
    hw_test.servo_run_ms = count_down(hw_test.servo_run_ms, elapsed);

    if (hw_test.esc_enable == 1U && !esc_started) start_escs();
    if (hw_test.esc_run_ms > HW_TEST_ESC_MAX_RUN_MS) hw_test.esc_run_ms = HW_TEST_ESC_MAX_RUN_MS;
    bool esc_run = esc_started && hw_test.esc_run_ms > 0U;
    if (esc_started) {
        for (int i = 0; i < 3; i++)
            set_esc(i, esc_run ? limit_esc(i, hw_test.esc_us[i]) : esc_stop[i]);
    }
    hw_test.esc_run_ms = count_down(hw_test.esc_run_ms, elapsed);

    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, hw_test.buzzer_ms > 0U ? BUZZER_ON_PULSE : 0U);
    hw_test.buzzer_ms = (uint16_t)count_down(hw_test.buzzer_ms, elapsed);
}
