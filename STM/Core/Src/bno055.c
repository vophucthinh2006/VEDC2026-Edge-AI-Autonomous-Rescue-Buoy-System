#include "bno055.h"

#define BNO055_ADDRESS             (0x28U << 1U)
#define BNO055_CHIP_ID_REG         0x00U
#define BNO055_PAGE_ID_REG         0x07U
#define BNO055_OPR_MODE_REG        0x3DU
#define BNO055_PWR_MODE_REG        0x3EU
#define BNO055_EULER_H_LSB_REG     0x1AU
#define BNO055_CALIB_STAT_REG      0x35U
#define BNO055_CONFIG_MODE         0x00U
#define BNO055_NDOF_MODE           0x0CU

static bool write_register(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(hi2c, BNO055_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &value, 1U, 50U) == HAL_OK;
}

bool BNO055_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t chip_id = 0U;
    if (HAL_I2C_Mem_Read(hi2c, BNO055_ADDRESS, BNO055_CHIP_ID_REG, I2C_MEMADD_SIZE_8BIT, &chip_id, 1U, 100U) != HAL_OK || chip_id != 0xA0U) {
        return false;
    }
    if (!write_register(hi2c, BNO055_OPR_MODE_REG, BNO055_CONFIG_MODE)) return false;
    HAL_Delay(25U);
    if (!write_register(hi2c, BNO055_PAGE_ID_REG, 0U)) return false;
    if (!write_register(hi2c, BNO055_PWR_MODE_REG, 0U)) return false; /* normal power */
    HAL_Delay(10U);
    if (!write_register(hi2c, BNO055_OPR_MODE_REG, BNO055_NDOF_MODE)) return false;
    HAL_Delay(20U);
    return true;
}

bool BNO055_ReadEuler(I2C_HandleTypeDef *hi2c, bno055_euler_t *out) {
    uint8_t raw[6] = {0};
    uint8_t calibration = 0U;
    if (HAL_I2C_Mem_Read(hi2c, BNO055_ADDRESS, BNO055_EULER_H_LSB_REG, I2C_MEMADD_SIZE_8BIT, raw, sizeof(raw), 20U) != HAL_OK) {
        out->valid = false;
        return false;
    }
    (void)HAL_I2C_Mem_Read(hi2c, BNO055_ADDRESS, BNO055_CALIB_STAT_REG, I2C_MEMADD_SIZE_8BIT, &calibration, 1U, 20U);
    int16_t heading = (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8U));
    int16_t roll = (int16_t)((uint16_t)raw[2] | ((uint16_t)raw[3] << 8U));
    int16_t pitch = (int16_t)((uint16_t)raw[4] | ((uint16_t)raw[5] << 8U));
    out->heading_deg = (float)heading / 16.0f;
    out->roll_deg = (float)roll / 16.0f;
    out->pitch_deg = (float)pitch / 16.0f;
    out->calibration = (calibration >> 6U) & 0x03U;
    out->valid = true;
    return true;
}
