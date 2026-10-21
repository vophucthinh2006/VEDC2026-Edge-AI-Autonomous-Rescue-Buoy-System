#include "i2c_bus.h"

/* The pins of I2C1, as Core/Src/i2c.c sets them up. */
#define SCL_PORT  GPIOB
#define SCL_PIN   GPIO_PIN_8
#define SDA_PORT  GPIOB
#define SDA_PIN   GPIO_PIN_9

#define I2C_RECOVER_PERIOD_MS  50U

volatile i2c_bus_debug_t i2c_bus_debug;

/* About 5 us at 168 MHz (more at -O0): a 100 kHz bit, slow enough for any slave. */
static void half_bit(void)
{
    for (volatile uint32_t i = 0U; i < 200U; i++) { }
}

static void scl(GPIO_PinState level) { HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, level); half_bit(); }
static void sda(GPIO_PinState level) { HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, level); half_bit(); }

void I2cBus_Recover(I2C_HandleTypeDef *hi2c)
{
    if (i2c_bus_debug.recoveries < 0xFFFFU) i2c_bus_debug.recoveries++;
    (void)HAL_I2C_DeInit(hi2c);   /* peripheral off, pins back to inputs (HAL_I2C_MspDeInit) */

    GPIO_InitTypeDef pin = { 0 };
    pin.Mode = GPIO_MODE_OUTPUT_OD;
    pin.Pull = GPIO_PULLUP;
    pin.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_SET);
    pin.Pin = SCL_PIN;
    HAL_GPIO_Init(SCL_PORT, &pin);
    pin.Pin = SDA_PIN;
    HAL_GPIO_Init(SDA_PORT, &pin);
    half_bit();

    /* A slave stopped mid-byte holds SDA low until it has clocked the rest of it out. */
    for (int i = 0; i < 9 && HAL_GPIO_ReadPin(SDA_PORT, SDA_PIN) == GPIO_PIN_RESET; i++) {
        scl(GPIO_PIN_RESET);
        scl(GPIO_PIN_SET);
    }
    /* STOP: SDA rises while SCL is high; every slave then lets go of the bus. */
    scl(GPIO_PIN_RESET);
    sda(GPIO_PIN_RESET);
    scl(GPIO_PIN_SET);
    sda(GPIO_PIN_SET);
    if (HAL_GPIO_ReadPin(SDA_PORT, SDA_PIN) == GPIO_PIN_RESET && i2c_bus_debug.sda_stuck < 0xFFFFU) i2c_bus_debug.sda_stuck++;

    /* HAL_I2C_Init gives the pins back to I2C1 (HAL_I2C_MspInit) and resets the peripheral (SWRST),
       which clears a BUSY flag left set. hi2c->Init still holds CubeMX's settings. Not MX_I2C1_Init:
       its Error_Handler would stop the board for good. */
    if (HAL_I2C_Init(hi2c) != HAL_OK && i2c_bus_debug.init_fails < 0xFFFFU) i2c_bus_debug.init_fails++;
}

bool I2cBus_Ready(I2C_HandleTypeDef *hi2c, uint32_t now_ms)
{
    static uint32_t last_recover_ms;
    static bool recovered_once;
    if (hi2c->State == HAL_I2C_STATE_READY && __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == RESET) return true;
    if (i2c_bus_debug.busy_ticks < 0xFFFFU) i2c_bus_debug.busy_ticks++;
    if (!recovered_once || (uint32_t)(now_ms - last_recover_ms) >= I2C_RECOVER_PERIOD_MS) {
        recovered_once = true;
        last_recover_ms = now_ms;
        I2cBus_Recover(hi2c);
    }
    return false;
}
