#include "ina226.h"

#define REG_CONFIG       0x00U
#define REG_BUS_VOLTAGE  0x02U
#define REG_CURRENT      0x04U
#define REG_CALIBRATION  0x05U
#define REG_MANUFACTURER 0xFEU
#define MANUFACTURER_TI  0x5449U

/* AVG 16 (010), VBUSCT 1.1 ms (100), VSHCT 1.1 ms (100), MODE shunt and bus continuous (111):
   a fresh averaged value every ~35 ms. */
#define CONFIG_VALUE     0x4527U
#define I2C_TIMEOUT_MS   5U        /* shares the bus with the BNO055 inside the 10 ms control tick */

static bool read_reg(ina226_t *dev, uint8_t reg, uint16_t *value)
{
    uint8_t data[2];
    if (HAL_I2C_Mem_Read(dev->hi2c, (uint16_t)(dev->address << 1), reg, I2C_MEMADD_SIZE_8BIT, data, 2U, I2C_TIMEOUT_MS) != HAL_OK)
        return false;
    *value = (uint16_t)((data[0] << 8) | data[1]);
    return true;
}

static bool write_reg(ina226_t *dev, uint8_t reg, uint16_t value)
{
    uint8_t data[2] = { (uint8_t)(value >> 8), (uint8_t)value };
    return HAL_I2C_Mem_Write(dev->hi2c, (uint16_t)(dev->address << 1), reg, I2C_MEMADD_SIZE_8BIT, data, 2U, I2C_TIMEOUT_MS) == HAL_OK;
}

bool INA226_Init(ina226_t *dev, I2C_HandleTypeDef *hi2c, uint8_t address, float shunt_ohm, float max_current_a)
{
    dev->hi2c = hi2c;
    dev->address = address;
    /* Section 7.5: Current_LSB = max current / 2^15, CAL = 0.00512 / (Current_LSB * R_shunt). */
    dev->current_lsb_a = max_current_a / 32768.0f;
    float cal = 0.00512f / (dev->current_lsb_a * shunt_ohm);
    uint16_t id;
    if (!read_reg(dev, REG_MANUFACTURER, &id) || id != MANUFACTURER_TI) return false;
    return write_reg(dev, REG_CONFIG, CONFIG_VALUE) &&
           write_reg(dev, REG_CALIBRATION, (uint16_t)(cal > 65535.0f ? 65535.0f : cal));
}

bool INA226_Read(ina226_t *dev, float *volts, float *amps)
{
    uint16_t bus, current;
    if (!read_reg(dev, REG_BUS_VOLTAGE, &bus) || !read_reg(dev, REG_CURRENT, &current)) return false;
    *volts = (float)bus * 0.00125f;
    *amps = (float)(int16_t)current * dev->current_lsb_a;
    return true;
}
