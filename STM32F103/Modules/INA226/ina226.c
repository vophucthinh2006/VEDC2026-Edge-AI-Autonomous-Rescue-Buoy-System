#include "ina226.h"

#define REG_BUS_VOLTAGE   0x02U
#define REG_CURRENT       0x04U
#define REG_CALIBRATION   0x05U
#define REG_MANUFACTURER  0xFEU
#define MANUFACTURER_ID   0x5449U   /* "TI" */
#define I2C_TIMEOUT_MS    5U

static bool read_reg(ina226_t *dev, uint8_t reg, uint16_t *value) {
    uint8_t raw[2];
    if (HAL_I2C_Mem_Read(dev->hi2c, (uint16_t)(dev->address << 1), reg, I2C_MEMADD_SIZE_8BIT, raw, 2U, I2C_TIMEOUT_MS) != HAL_OK) return false;
    *value = (uint16_t)((raw[0] << 8) | raw[1]);   /* registers are big-endian */
    return true;
}

bool INA226_Init(ina226_t *dev) {
    uint16_t id = 0U;
    if (!read_reg(dev, REG_MANUFACTURER, &id) || id != MANUFACTURER_ID) return false;
    uint8_t raw[2] = { (uint8_t)(dev->calibration >> 8), (uint8_t)dev->calibration };
    return HAL_I2C_Mem_Write(dev->hi2c, (uint16_t)(dev->address << 1), REG_CALIBRATION, I2C_MEMADD_SIZE_8BIT, raw, 2U, I2C_TIMEOUT_MS) == HAL_OK;
}

bool INA226_Read(ina226_t *dev, uint16_t *bus_mv, int32_t *current_ma) {
    uint16_t bus = 0U, current = 0U;
    if (!read_reg(dev, REG_BUS_VOLTAGE, &bus) || !read_reg(dev, REG_CURRENT, &current)) return false;
    *bus_mv = (uint16_t)(((uint32_t)bus * 5U) / 4U);                       /* 1.25 mV per count */
    *current_ma = ((int32_t)(int16_t)current * (int32_t)dev->current_lsb_ua) / 1000;
    return true;
}
