#include "lights.h"
#include "app_config.h"
#include "main.h"

#define LEVEL(on) (((on) != 0) == (RELAY_ON_LEVEL != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET)

static uint8_t lights;

void Lights_Init(void)
{
    Lights_Set(0U);
}

void Lights_Set(uint8_t mask)
{
    lights = (uint8_t)(mask & 0x03U);
    HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, LEVEL(lights & 0x01U));
    HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, LEVEL(lights & 0x02U));
}

uint8_t Lights_Get(void)
{
    return lights;
}
