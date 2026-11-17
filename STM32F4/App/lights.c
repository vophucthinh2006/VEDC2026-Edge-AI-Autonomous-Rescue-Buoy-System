#include "lights.h"
#include "app_config.h"
#include "main.h"

#define LEVEL(on) (((on) != 0) == (RELAY_ON_LEVEL != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET)

static uint8_t lights;       /* what the relays are driven to */
static uint8_t shore;        /* the dashboard's mask, kept while the phone owns the lights */
static bool manual;

static void drive(uint8_t mask)
{
    lights = (uint8_t)(mask & 0x03U);
    HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, LEVEL(lights & 0x01U));
    HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, LEVEL(lights & 0x02U));
}

void Lights_Init(void)
{
    Lights_Set(0U);
}

void Lights_Set(uint8_t mask)
{
    shore = (uint8_t)(mask & 0x03U);
    if (!manual) drive(shore);
}

void Lights_Manual(bool on, uint8_t mask)
{
    uint8_t want = on ? (uint8_t)(mask & 0x03U) : shore;
    if (on != manual || want != lights) drive(want);
    manual = on;
}

uint8_t Lights_Get(void)
{
    return lights;
}
