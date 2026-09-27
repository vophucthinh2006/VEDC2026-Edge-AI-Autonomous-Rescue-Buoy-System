#include "buzzer.h"
#include "tim.h"

#define BUZZER_ON_PULSE  125U   /* half of TIM8 ARR 249: 4 kHz square wave */
#define BEEP_MS          80U

static uint8_t pending;   /* on/off phases left; even and non-zero = sounding */
static uint32_t phase_ms;

void Buzzer_Init(void) {
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0U);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
}

void Buzzer_Beep(uint8_t count) {
    pending = (uint8_t)(2U * count);
    phase_ms = HAL_GetTick();
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, BUZZER_ON_PULSE);
}

void Buzzer_Tick(uint32_t now_ms) {
    if (pending == 0U || (uint32_t)(now_ms - phase_ms) < BEEP_MS) return;
    phase_ms = now_ms;
    pending--;
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (pending % 2U == 0U && pending > 0U) ? BUZZER_ON_PULSE : 0U);
}
