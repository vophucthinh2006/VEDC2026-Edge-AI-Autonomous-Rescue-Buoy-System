#include "actuators.h"
#include "app_config.h"

static uint32_t clamp_us(uint32_t value, uint32_t low, uint32_t high) { return value < low ? low : (value > high ? high : value); }
static void pwm(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t microseconds) { __HAL_TIM_SET_COMPARE(timer, channel, microseconds); }

void Actuators_Init(actuator_driver_t *driver, TIM_HandleTypeDef *tim3, TIM_HandleTypeDef *tim4) {
    driver->tim3 = tim3; driver->tim4 = tim4;
    HAL_TIM_PWM_Start(tim3, TIM_CHANNEL_1); HAL_TIM_PWM_Start(tim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(tim3, TIM_CHANNEL_3); HAL_TIM_PWM_Start(tim3, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(tim4, TIM_CHANNEL_1); HAL_TIM_PWM_Start(tim4, TIM_CHANNEL_2);
    Actuators_Stop(driver);
}

void Actuators_Stop(const actuator_driver_t *driver) {
    pwm(driver->tim3, TIM_CHANNEL_1, ESC_STOP_US); pwm(driver->tim3, TIM_CHANNEL_2, ESC_STOP_US); pwm(driver->tim3, TIM_CHANNEL_3, ESC_STOP_US);
    pwm(driver->tim3, TIM_CHANNEL_4, SERVO_CENTER_US); pwm(driver->tim4, TIM_CHANNEL_1, SERVO_CENTER_US); pwm(driver->tim4, TIM_CHANNEL_2, SERVO_CENTER_US);
}

void Actuators_SetAuto(const actuator_driver_t *driver, float throttle, float yaw) {
#if ACTUATORS_ENABLED
    if (throttle < 0.0f) throttle = 0.0f; if (throttle > 1.0f) throttle = 1.0f;
    if (yaw < -0.35f) yaw = -0.35f; if (yaw > 0.35f) yaw = 0.35f;
    float left = throttle - yaw, right = throttle + yaw;
    if (left < 0.0f) left = 0.0f; if (left > 1.0f) left = 1.0f;
    if (right < 0.0f) right = 0.0f; if (right > 1.0f) right = 1.0f;
    pwm(driver->tim3, TIM_CHANNEL_1, ESC_STOP_US + (uint32_t)(left * (ESC_MAX_US - ESC_STOP_US)));
    pwm(driver->tim3, TIM_CHANNEL_2, ESC_STOP_US + (uint32_t)(right * (ESC_MAX_US - ESC_STOP_US)));
    pwm(driver->tim3, TIM_CHANNEL_3, ESC_STOP_US + (uint32_t)(throttle * (ESC_MAX_US - ESC_STOP_US)));
#else
    (void)throttle; (void)yaw; Actuators_Stop(driver);
#endif
}

void Actuators_SetManual(const actuator_driver_t *driver, uint16_t throttle_us, uint16_t yaw_us) {
#if ACTUATORS_ENABLED
    uint32_t throttle = clamp_us(throttle_us, ESC_STOP_US, ESC_MAX_US);
    int32_t differential = (int32_t)clamp_us(yaw_us, SERVO_MIN_US, SERVO_MAX_US) - SERVO_CENTER_US;
    pwm(driver->tim3, TIM_CHANNEL_1, clamp_us((uint32_t)((int32_t)throttle - differential), ESC_STOP_US, ESC_MAX_US));
    pwm(driver->tim3, TIM_CHANNEL_2, clamp_us((uint32_t)((int32_t)throttle + differential), ESC_STOP_US, ESC_MAX_US));
    pwm(driver->tim3, TIM_CHANNEL_3, throttle);
#else
    (void)throttle_us; (void)yaw_us; Actuators_Stop(driver);
#endif
}
