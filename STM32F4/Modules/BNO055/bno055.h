/**
  ******************************************************************************
  * @file    bno055.h
  * @brief   Bosch BNO055 9-axis absolute orientation sensor driver (I2C, HAL).
  *
  *          Reference: Bosch Sensortec BNO055 data sheet
  *          BST-BNO055-DS000-18, revision 1.8, October 2021.
  *
  *          Usage:
  *            1. Fill hi2c, address, rst_port/rst_pin, mode, use_ext_crystal,
  *               calib_profile (optional, see bno055_calib_profile.h).
  *            2. Call BNO055_Init() once after MX_I2C1_Init().
  *            3. Call BNO055_ReadEuler() periodically (fusion output 100 Hz).
  *            4. Once CALIB_STAT is fully calibrated, BNO055_ReadCalibProfile()
  *               gives the profile to store (section 3.11.5).
  ******************************************************************************
  */

#ifndef BNO055_H
#define BNO055_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

/* I2C 7-bit slave address, selected by COM3 / ADR pin (Table 4-7) */
#define BNO055_I2C_ADDR_COM3_LOW    0x28U
#define BNO055_I2C_ADDR_COM3_HIGH   0x29U   /* COM3 has internal pull-up: default */

/* Calibration profile: sensor offsets + radius, ACC_OFFSET_X_LSB (0x55) .. MAG_RADIUS_MSB (0x6A) */
#define BNO055_CALIB_PROFILE_SIZE   22U

/**
  * @brief Operation modes, OPR_MODE register value (Table 3-5)
  */
typedef enum
{
  BNO055_OPR_MODE_CONFIG       = 0x00U,
  /* Non-fusion modes */
  BNO055_OPR_MODE_ACCONLY      = 0x01U,
  BNO055_OPR_MODE_MAGONLY      = 0x02U,
  BNO055_OPR_MODE_GYROONLY     = 0x03U,
  BNO055_OPR_MODE_ACCMAG       = 0x04U,
  BNO055_OPR_MODE_ACCGYRO      = 0x05U,
  BNO055_OPR_MODE_MAGGYRO      = 0x06U,
  BNO055_OPR_MODE_AMG          = 0x07U,
  /* Fusion modes (Euler angles available) */
  BNO055_OPR_MODE_IMU          = 0x08U,
  BNO055_OPR_MODE_COMPASS      = 0x09U,
  BNO055_OPR_MODE_M4G          = 0x0AU,
  BNO055_OPR_MODE_NDOF_FMC_OFF = 0x0BU,
  BNO055_OPR_MODE_NDOF         = 0x0CU
} BNO055_OprMode_t;

/**
  * @brief Driver return codes
  */
typedef enum
{
  BNO055_OK = 0,
  BNO055_ERR_PARAM,     /* NULL handle / pointer */
  BNO055_ERR_I2C,       /* HAL I2C transfer failed (NACK, bus error, timeout) */
  BNO055_ERR_TIMEOUT,   /* device did not become ready in time */
  BNO055_ERR_CHIP_ID,   /* CHIP_ID != 0xA0 */
  BNO055_ERR_MODE,      /* OPR_MODE read-back mismatch */
  BNO055_ERR_SYS,       /* SYS_STATUS reports system error, see sys_err */
  BNO055_ERR_PROFILE    /* calibration profile outside data sheet ranges (3.6.4) */
} BNO055_Status_t;

/**
  * @brief Driver handle
  */
typedef struct
{
  /* Configuration, set by user before BNO055_Init() */
  I2C_HandleTypeDef *hi2c;           /* I2C peripheral handle */
  uint16_t           address;        /* 7-bit address, BNO055_I2C_ADDR_xxx */
  GPIO_TypeDef      *rst_port;       /* nRESET port, NULL = software reset */
  uint16_t           rst_pin;        /* nRESET pin */
  BNO055_OprMode_t   mode;           /* operation mode entered by Init */
  bool               use_ext_crystal;/* select external 32.768 kHz crystal */
  const uint8_t     *calib_profile;  /* BNO055_CALIB_PROFILE_SIZE bytes written by Init, NULL = none */

  /* Status, filled by driver */
  bool               ext_crystal_active; /* CLK_SEL accepted by device */
  uint8_t            sys_err;            /* SYS_ERR code when BNO055_ERR_SYS */
} BNO055_HandleTypeDef;

/**
  * @brief Euler angles in degrees (Windows orientation format, Table 3-13)
  */
typedef struct
{
  float roll;   /* -90 .. +90   */
  float pitch;  /* -180 .. +180 */
  float yaw;    /* 0 .. 360, heading */
} BNO055_Euler_t;

/**
  * @brief Calibration status, 0 = not calibrated .. 3 = fully calibrated (4.3.54)
  */
typedef struct
{
  uint8_t sys;
  uint8_t gyr;
  uint8_t acc;
  uint8_t mag;
} BNO055_CalibStatus_t;

BNO055_Status_t BNO055_Init(BNO055_HandleTypeDef *hbno);
BNO055_Status_t BNO055_ReadEuler(BNO055_HandleTypeDef *hbno, BNO055_Euler_t *euler);
BNO055_Status_t BNO055_ReadCalibStatus(BNO055_HandleTypeDef *hbno, BNO055_CalibStatus_t *calib);
BNO055_Status_t BNO055_ReadCalibProfile(BNO055_HandleTypeDef *hbno, uint8_t *profile);
BNO055_Status_t BNO055_WriteCalibProfile(BNO055_HandleTypeDef *hbno, const uint8_t *profile);

#ifdef __cplusplus
}
#endif

#endif /* BNO055_H */
