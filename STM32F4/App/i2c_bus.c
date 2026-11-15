#include "i2c_bus.h"

/* Pin pairs as Core/Src/i2c.c sets them up. I2C3 is the BNO055 bus; I2C1 is
   retained for the INA226 battery monitor. */
typedef struct {
    GPIO_TypeDef *scl_port;
    uint16_t scl_pin;
    GPIO_TypeDef *sda_port;
    uint16_t sda_pin;
} i2c_pins_t;

#define I2C_RECOVER_PERIOD_MS  50U

volatile i2c_bus_debug_t i2c_bus_debug;

/* About 5 us at 168 MHz (more at -O0): a 100 kHz bit, slow enough for any slave. */
static void half_bit(void)
{
    for (volatile uint32_t i = 0U; i < 200U; i++) { }
}

static bool i2c_pins(const I2C_HandleTypeDef *hi2c, i2c_pins_t *pins)
{
    if (hi2c->Instance == I2C1) {
        *pins = (i2c_pins_t){ GPIOB, GPIO_PIN_8, GPIOB, GPIO_PIN_9 };
        return true;
    }
    if (hi2c->Instance == I2C3) {
        *pins = (i2c_pins_t){ GPIOA, GPIO_PIN_8, GPIOC, GPIO_PIN_9 };
        return true;
    }
    return false;
}

static unsigned int i2c_slot(const I2C_HandleTypeDef *hi2c)
{
    return hi2c->Instance == I2C3 ? 1U : 0U;
}

static void scl(const i2c_pins_t *pins, GPIO_PinState level) { HAL_GPIO_WritePin(pins->scl_port, pins->scl_pin, level); half_bit(); }
static void sda(const i2c_pins_t *pins, GPIO_PinState level) { HAL_GPIO_WritePin(pins->sda_port, pins->sda_pin, level); half_bit(); }

void I2cBus_Recover(I2C_HandleTypeDef *hi2c)
{
    i2c_pins_t pins;
    if (!i2c_pins(hi2c, &pins)) return;
    if (i2c_bus_debug.recoveries < 0xFFFFU) i2c_bus_debug.recoveries++;
    (void)HAL_I2C_DeInit(hi2c);   /* peripheral off, pins back to inputs (HAL_I2C_MspDeInit) */

    GPIO_InitTypeDef pin = { 0 };
    pin.Mode = GPIO_MODE_OUTPUT_OD;
    pin.Pull = GPIO_PULLUP;
    pin.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_WritePin(pins.scl_port, pins.scl_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(pins.sda_port, pins.sda_pin, GPIO_PIN_SET);
    pin.Pin = pins.scl_pin;
    HAL_GPIO_Init(pins.scl_port, &pin);
    pin.Pin = pins.sda_pin;
    HAL_GPIO_Init(pins.sda_port, &pin);
    half_bit();

    /* A slave stopped mid-byte holds SDA low until it has clocked the rest of it out. */
    for (int i = 0; i < 9 && HAL_GPIO_ReadPin(pins.sda_port, pins.sda_pin) == GPIO_PIN_RESET; i++) {
        scl(&pins, GPIO_PIN_RESET);
        scl(&pins, GPIO_PIN_SET);
    }
    /* STOP: SDA rises while SCL is high; every slave then lets go of the bus. */
    scl(&pins, GPIO_PIN_RESET);
    sda(&pins, GPIO_PIN_RESET);
    scl(&pins, GPIO_PIN_SET);
    sda(&pins, GPIO_PIN_SET);
    if (HAL_GPIO_ReadPin(pins.sda_port, pins.sda_pin) == GPIO_PIN_RESET && i2c_bus_debug.sda_stuck < 0xFFFFU) i2c_bus_debug.sda_stuck++;

    /* HAL_I2C_Init gives the pins back to their bus (HAL_I2C_MspInit) and resets the peripheral (SWRST),
       which clears a BUSY flag left set. hi2c->Init still holds CubeMX's settings. Not MX_I2Cx_Init:
       its Error_Handler would stop the board for good. */
    if (HAL_I2C_Init(hi2c) != HAL_OK && i2c_bus_debug.init_fails < 0xFFFFU) i2c_bus_debug.init_fails++;
}

bool I2cBus_Ready(I2C_HandleTypeDef *hi2c, uint32_t now_ms)
{
    static uint32_t last_recover_ms[2];
    static bool recovered_once[2];
    const unsigned int slot = i2c_slot(hi2c);
    if (hi2c->State == HAL_I2C_STATE_READY && __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == RESET) return true;
    if (i2c_bus_debug.busy_ticks < 0xFFFFU) i2c_bus_debug.busy_ticks++;
    if (!recovered_once[slot] || (uint32_t)(now_ms - last_recover_ms[slot]) >= I2C_RECOVER_PERIOD_MS) {
        recovered_once[slot] = true;
        last_recover_ms[slot] = now_ms;
        I2cBus_Recover(hi2c);
    }
    return false;
}
