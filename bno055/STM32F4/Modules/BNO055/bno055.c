/**
  ******************************************************************************
  * @file    bno055.c
  * @brief   Bosch BNO055 9-axis absolute orientation sensor driver (I2C, HAL).
  *
  *          Reference: Bosch Sensortec BNO055 data sheet
  *          BST-BNO055-DS000-18, revision 1.8, October 2021.
  ******************************************************************************
  */

#include "bno055.h"

/* Register map page 0 (section 4.2.1, Table 4-2) ----------------------------*/
#define BNO055_REG_CHIP_ID          0x00U
#define BNO055_REG_PAGE_ID          0x07U
#define BNO055_REG_EUL_HEADING_LSB  0x1AU   /* heading, roll, pitch: 0x1A..0x1F */
#define BNO055_REG_CALIB_STAT       0x35U
#define BNO055_REG_SYS_CLK_STATUS   0x38U
#define BNO055_REG_SYS_STATUS       0x39U
#define BNO055_REG_SYS_ERR          0x3AU
#define BNO055_REG_UNIT_SEL         0x3BU
#define BNO055_REG_OPR_MODE         0x3DU
#define BNO055_REG_PWR_MODE         0x3EU
#define BNO055_REG_SYS_TRIGGER      0x3FU
#define BNO055_REG_ACC_OFFSET_X_LSB 0x55U   /* calibration profile: 0x55..0x6A (3.6.4) */

/* Register values ------------------------------------------------------------*/
#define BNO055_CHIP_ID_VALUE        0xA0U   /* 4.3.1 */
#define BNO055_PAGE_0               0x00U   /* 4.3.8 */
#define BNO055_SYS_TRIGGER_CLK_SEL  0x80U   /* 4.3.63 bit7 */
#define BNO055_SYS_TRIGGER_RST_SYS  0x20U   /* 4.3.63 bit5 */
#define BNO055_ST_MAIN_CLK          0x01U   /* 4.3.57 bit0: 1 = clock being configured */
#define BNO055_SYS_STATUS_ERROR     0x01U   /* 4.3.58 */
#define BNO055_PWR_MODE_NORMAL      0x00U   /* Table 3-1 */
#define BNO055_OPR_MODE_MASK        0x0FU   /* 4.3.61 */

/* UNIT_SEL (4.3.60): Windows orientation, Celsius, degrees, dps, m/s^2 */
#define BNO055_UNIT_SEL_VALUE       0x00U

/* Euler angle representation: 1 degree = 16 LSB (Table 3-29) */
#define BNO055_EULER_LSB_PER_DEG    16.0f

/* Timing -----------------------------------------------------------------------*/
#define BNO055_T_RST_PULSE_MS       1U      /* nRESET low >= 20 ns (3.2) */
#define BNO055_T_POR_MS             650U    /* reset -> CONFIGMODE, typ (Table 0-2) */
#define BNO055_T_CONFIG_TO_OP_MS    7U      /* Table 3-6 */
#define BNO055_T_OP_TO_CONFIG_MS    19U     /* Table 3-6 */
#define BNO055_BOOT_TIMEOUT_MS      1000U   /* margin over T_POR for CHIP_ID polling */
#define BNO055_EXT_CLK_TIMEOUT_MS   1000U   /* crystal start-up ~600 ms (5.5.1) */
#define BNO055_POLL_INTERVAL_MS     10U
#define BNO055_I2C_TIMEOUT_MS       100U

/* Private functions ----------------------------------------------------------*/

/**
  * @brief  Write one register (I2C write access, Figure 6).
  */
static BNO055_Status_t BNO055_WriteReg(BNO055_HandleTypeDef *hbno, uint8_t reg, uint8_t value)
{
  if (HAL_I2C_Mem_Write(hbno->hi2c, (uint16_t)(hbno->address << 1), reg,
                        I2C_MEMADD_SIZE_8BIT, &value, 1U, BNO055_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return BNO055_ERR_I2C;
  }
  return BNO055_OK;
}

/**
  * @brief  Read consecutive registers (I2C multiple read with repeated start, Figure 7).
  *         A burst read keeps LSB/MSB pairs consistent (data register shadowing, 3.7).
  */
static BNO055_Status_t BNO055_ReadReg(BNO055_HandleTypeDef *hbno, uint8_t reg, uint8_t *buf, uint16_t len)
{
  if (HAL_I2C_Mem_Read(hbno->hi2c, (uint16_t)(hbno->address << 1), reg,
                       I2C_MEMADD_SIZE_8BIT, buf, len, BNO055_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return BNO055_ERR_I2C;
  }
  return BNO055_OK;
}

/**
  * @brief  Poll CHIP_ID until the device answers with 0xA0 after power-on / reset.
  */
static BNO055_Status_t BNO055_WaitChipId(BNO055_HandleTypeDef *hbno)
{
  uint32_t start = HAL_GetTick();
  bool responded = false;
  uint8_t chip_id = 0U;

  do
  {
    if (BNO055_ReadReg(hbno, BNO055_REG_CHIP_ID, &chip_id, 1U) == BNO055_OK)
    {
      if (chip_id == BNO055_CHIP_ID_VALUE)
      {
        return BNO055_OK;
      }
      responded = true;
    }
    HAL_Delay(BNO055_POLL_INTERVAL_MS);
  } while ((HAL_GetTick() - start) < BNO055_BOOT_TIMEOUT_MS);

  return responded ? BNO055_ERR_CHIP_ID : BNO055_ERR_TIMEOUT;
}

/**
  * @brief  Reset the device: nRESET pulse if wired, otherwise SYS_TRIGGER.RST_SYS (3.2).
  *         The device restarts in CONFIGMODE with default register values.
  */
static BNO055_Status_t BNO055_Reset(BNO055_HandleTypeDef *hbno)
{
  BNO055_Status_t status;

  if (hbno->rst_port != NULL)
  {
    HAL_GPIO_WritePin(hbno->rst_port, hbno->rst_pin, GPIO_PIN_RESET);
    HAL_Delay(BNO055_T_RST_PULSE_MS);
    HAL_GPIO_WritePin(hbno->rst_port, hbno->rst_pin, GPIO_PIN_SET);
  }
  else
  {
    status = BNO055_WaitChipId(hbno);
    if (status != BNO055_OK)
    {
      return status;
    }
    status = BNO055_WriteReg(hbno, BNO055_REG_SYS_TRIGGER, BNO055_SYS_TRIGGER_RST_SYS);
    if (status != BNO055_OK)
    {
      return status;
    }
  }

  HAL_Delay(BNO055_T_POR_MS);
  return BNO055_WaitChipId(hbno);
}

/**
  * @brief  Write OPR_MODE and wait the mode switching time (Table 3-6).
  */
static BNO055_Status_t BNO055_SetOprMode(BNO055_HandleTypeDef *hbno, BNO055_OprMode_t mode)
{
  BNO055_Status_t status = BNO055_WriteReg(hbno, BNO055_REG_OPR_MODE, (uint8_t)mode);
  if (status != BNO055_OK)
  {
    return status;
  }

  HAL_Delay((mode == BNO055_OPR_MODE_CONFIG) ? BNO055_T_OP_TO_CONFIG_MS : BNO055_T_CONFIG_TO_OP_MS);
  return BNO055_OK;
}

/**
  * @brief  Wait until SYS_CLK_STATUS.ST_MAIN_CLK is cleared (clock source free to configure).
  */
static BNO055_Status_t BNO055_WaitClockIdle(BNO055_HandleTypeDef *hbno)
{
  uint32_t start = HAL_GetTick();
  uint8_t clk_status;

  do
  {
    if (BNO055_ReadReg(hbno, BNO055_REG_SYS_CLK_STATUS, &clk_status, 1U) != BNO055_OK)
    {
      return BNO055_ERR_I2C;
    }
    if ((clk_status & BNO055_ST_MAIN_CLK) == 0U)
    {
      return BNO055_OK;
    }
    HAL_Delay(BNO055_POLL_INTERVAL_MS);
  } while ((HAL_GetTick() - start) < BNO055_EXT_CLK_TIMEOUT_MS);

  return BNO055_ERR_TIMEOUT;
}

/**
  * @brief  Select the external 32.768 kHz crystal (5.5.1). Must be called in CONFIGMODE.
  *         If the crystal signal is bad the device falls back to the internal clock
  *         and clears CLK_SEL; this is reported in ext_crystal_active, not as an error.
  */
static BNO055_Status_t BNO055_SelectExtCrystal(BNO055_HandleTypeDef *hbno)
{
  BNO055_Status_t status;
  uint8_t sys_trigger;

  hbno->ext_crystal_active = false;

  status = BNO055_WaitClockIdle(hbno);
  if (status != BNO055_OK)
  {
    return status;
  }
  status = BNO055_WriteReg(hbno, BNO055_REG_SYS_TRIGGER, BNO055_SYS_TRIGGER_CLK_SEL);
  if (status != BNO055_OK)
  {
    return status;
  }
  status = BNO055_WaitClockIdle(hbno);
  if (status != BNO055_OK)
  {
    return status;
  }
  status = BNO055_ReadReg(hbno, BNO055_REG_SYS_TRIGGER, &sys_trigger, 1U);
  if (status != BNO055_OK)
  {
    return status;
  }

  hbno->ext_crystal_active = ((sys_trigger & BNO055_SYS_TRIGGER_CLK_SEL) != 0U);
  return BNO055_OK;
}

/* Calibration profile layout and ranges (3.6.4, Tables 3-16, 3-21, 3-24) */
#define BNO055_PROFILE_ACC_OFFSET_IDX   0U
#define BNO055_PROFILE_MAG_OFFSET_IDX   6U
#define BNO055_PROFILE_GYR_OFFSET_IDX   12U
#define BNO055_PROFILE_ACC_RADIUS_IDX   18U
#define BNO055_PROFILE_MAG_RADIUS_IDX   20U
#define BNO055_ACC_OFFSET_LIMIT         500
#define BNO055_MAG_OFFSET_LIMIT         6400
#define BNO055_GYR_OFFSET_LIMIT         2000
#define BNO055_ACC_RADIUS_LIMIT         2048
#define BNO055_MAG_RADIUS_MIN           144
#define BNO055_MAG_RADIUS_MAX           1280

static int16_t BNO055_ProfileS16(const uint8_t *profile, uint8_t idx)
{
  return (int16_t)(((uint16_t)profile[idx + 1U] << 8) | profile[idx]);
}

static bool BNO055_InRange(int16_t value, int16_t min, int16_t max)
{
  return (value >= min) && (value <= max);
}

/**
  * @brief  Check offsets and radius against the ranges given in section 3.6.4.
  *         Rejects corrupted profiles, e.g. 0xFF bytes from an aborted I2C read.
  */
static bool BNO055_IsProfileValid(const uint8_t *profile)
{
  for (uint8_t axis = 0U; axis < 3U; axis++)
  {
    uint8_t off = (uint8_t)(2U * axis);

    if (!BNO055_InRange(BNO055_ProfileS16(profile, BNO055_PROFILE_ACC_OFFSET_IDX + off),
                        -BNO055_ACC_OFFSET_LIMIT, BNO055_ACC_OFFSET_LIMIT) ||
        !BNO055_InRange(BNO055_ProfileS16(profile, BNO055_PROFILE_MAG_OFFSET_IDX + off),
                        -BNO055_MAG_OFFSET_LIMIT, BNO055_MAG_OFFSET_LIMIT) ||
        !BNO055_InRange(BNO055_ProfileS16(profile, BNO055_PROFILE_GYR_OFFSET_IDX + off),
                        -BNO055_GYR_OFFSET_LIMIT, BNO055_GYR_OFFSET_LIMIT))
    {
      return false;
    }
  }

  return BNO055_InRange(BNO055_ProfileS16(profile, BNO055_PROFILE_ACC_RADIUS_IDX),
                        -BNO055_ACC_RADIUS_LIMIT, BNO055_ACC_RADIUS_LIMIT) &&
         BNO055_InRange(BNO055_ProfileS16(profile, BNO055_PROFILE_MAG_RADIUS_IDX),
                        BNO055_MAG_RADIUS_MIN, BNO055_MAG_RADIUS_MAX);
}

/**
  * @brief  Write the calibration profile registers in ascending order. Must be in CONFIGMODE.
  *         Offsets / radius are applied when their last (MSB) byte is written (3.6.4),
  *         which ascending order guarantees.
  */
static BNO055_Status_t BNO055_WriteProfileRegs(BNO055_HandleTypeDef *hbno, const uint8_t *profile)
{
  BNO055_Status_t status;

  for (uint8_t i = 0U; i < BNO055_CALIB_PROFILE_SIZE; i++)
  {
    status = BNO055_WriteReg(hbno, (uint8_t)(BNO055_REG_ACC_OFFSET_X_LSB + i), profile[i]);
    if (status != BNO055_OK)
    {
      return status;
    }
  }
  return BNO055_OK;
}

/* Public functions -----------------------------------------------------------*/

/**
  * @brief  Reset and configure the BNO055, then enter hbno->mode.
  *         Sequence: reset -> page 0 -> CONFIGMODE -> PWR_MODE normal
  *         -> (external crystal) -> UNIT_SEL -> (calibration profile, 3.11.5)
  *         -> operation mode -> check SYS_STATUS.
  * @note   Blocking, takes about 0.7 s (up to ~2 s with external crystal).
  */
BNO055_Status_t BNO055_Init(BNO055_HandleTypeDef *hbno)
{
  BNO055_Status_t status;
  uint8_t value;

  if ((hbno == NULL) || (hbno->hi2c == NULL))
  {
    return BNO055_ERR_PARAM;
  }

  hbno->ext_crystal_active = false;
  hbno->sys_err = 0U;

  status = BNO055_Reset(hbno);
  if (status != BNO055_OK)
  {
    return status;
  }

  status = BNO055_WriteReg(hbno, BNO055_REG_PAGE_ID, BNO055_PAGE_0);
  if (status != BNO055_OK)
  {
    return status;
  }

  /* Writable configuration registers can only be changed in CONFIGMODE (3.3.1) */
  status = BNO055_SetOprMode(hbno, BNO055_OPR_MODE_CONFIG);
  if (status != BNO055_OK)
  {
    return status;
  }

  status = BNO055_WriteReg(hbno, BNO055_REG_PWR_MODE, BNO055_PWR_MODE_NORMAL);
  if (status != BNO055_OK)
  {
    return status;
  }

  if (hbno->use_ext_crystal)
  {
    status = BNO055_SelectExtCrystal(hbno);
    if (status != BNO055_OK)
    {
      return status;
    }
  }

  status = BNO055_WriteReg(hbno, BNO055_REG_UNIT_SEL, BNO055_UNIT_SEL_VALUE);
  if (status != BNO055_OK)
  {
    return status;
  }

  if (hbno->calib_profile != NULL)
  {
    if (!BNO055_IsProfileValid(hbno->calib_profile))
    {
      return BNO055_ERR_PROFILE;
    }
    status = BNO055_WriteProfileRegs(hbno, hbno->calib_profile);
    if (status != BNO055_OK)
    {
      return status;
    }
  }

  status = BNO055_SetOprMode(hbno, hbno->mode);
  if (status != BNO055_OK)
  {
    return status;
  }

  status = BNO055_ReadReg(hbno, BNO055_REG_OPR_MODE, &value, 1U);
  if (status != BNO055_OK)
  {
    return status;
  }
  if ((value & BNO055_OPR_MODE_MASK) != (uint8_t)hbno->mode)
  {
    return BNO055_ERR_MODE;
  }

  status = BNO055_ReadReg(hbno, BNO055_REG_SYS_STATUS, &value, 1U);
  if (status != BNO055_OK)
  {
    return status;
  }
  if (value == BNO055_SYS_STATUS_ERROR)
  {
    (void)BNO055_ReadReg(hbno, BNO055_REG_SYS_ERR, &hbno->sys_err, 1U);
    return BNO055_ERR_SYS;
  }

  return BNO055_OK;
}

/**
  * @brief  Read fused orientation as Euler angles in degrees (3.6.5.4).
  *         Only valid in fusion modes; data reads as zero in CONFIGMODE.
  */
BNO055_Status_t BNO055_ReadEuler(BNO055_HandleTypeDef *hbno, BNO055_Euler_t *euler)
{
  uint8_t buf[6];
  BNO055_Status_t status;

  if ((hbno == NULL) || (euler == NULL))
  {
    return BNO055_ERR_PARAM;
  }

  /* EUL_Heading LSB/MSB, EUL_Roll LSB/MSB, EUL_Pitch LSB/MSB (Table 3-28) */
  status = BNO055_ReadReg(hbno, BNO055_REG_EUL_HEADING_LSB, buf, sizeof(buf));
  if (status != BNO055_OK)
  {
    return status;
  }

  int16_t heading = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
  int16_t roll    = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
  int16_t pitch   = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);

  euler->yaw   = (float)heading / BNO055_EULER_LSB_PER_DEG;
  euler->roll  = (float)roll    / BNO055_EULER_LSB_PER_DEG;
  euler->pitch = (float)pitch   / BNO055_EULER_LSB_PER_DEG;

  return BNO055_OK;
}

/**
  * @brief  Read CALIB_STAT (4.3.54). Heading from NDOF is reliable once mag >= 2 / sys = 3.
  */
BNO055_Status_t BNO055_ReadCalibStatus(BNO055_HandleTypeDef *hbno, BNO055_CalibStatus_t *calib)
{
  uint8_t value;
  BNO055_Status_t status;

  if ((hbno == NULL) || (calib == NULL))
  {
    return BNO055_ERR_PARAM;
  }

  status = BNO055_ReadReg(hbno, BNO055_REG_CALIB_STAT, &value, 1U);
  if (status != BNO055_OK)
  {
    return status;
  }

  calib->sys = (value >> 6) & 0x03U;
  calib->gyr = (value >> 4) & 0x03U;
  calib->acc = (value >> 2) & 0x03U;
  calib->mag = value & 0x03U;

  return BNO055_OK;
}

/**
  * @brief  Read the calibration profile (offsets + radius) into profile[BNO055_CALIB_PROFILE_SIZE].
  *         The data sheet requires full calibration and CONFIGMODE for reading (3.11.5),
  *         so fusion is halted for ~26 ms and hbno->mode is restored afterwards.
  *         Read in per-sensor blocks (3.6.4) and range-checked: BNO055_ERR_PROFILE if invalid.
  */
BNO055_Status_t BNO055_ReadCalibProfile(BNO055_HandleTypeDef *hbno, uint8_t *profile)
{
  BNO055_Status_t status;
  BNO055_Status_t mode_status;

  if ((hbno == NULL) || (profile == NULL))
  {
    return BNO055_ERR_PARAM;
  }

  /* acc offsets, mag offsets, gyr offsets, acc + mag radius */
  static const uint8_t block_len[] = { 6U, 6U, 6U, 4U };
  uint8_t pos = 0U;

  status = BNO055_SetOprMode(hbno, BNO055_OPR_MODE_CONFIG);
  for (uint8_t i = 0U; (status == BNO055_OK) && (i < sizeof(block_len)); i++)
  {
    status = BNO055_ReadReg(hbno, (uint8_t)(BNO055_REG_ACC_OFFSET_X_LSB + pos), &profile[pos], block_len[i]);
    pos = (uint8_t)(pos + block_len[i]);
  }
  if ((status == BNO055_OK) && !BNO055_IsProfileValid(profile))
  {
    status = BNO055_ERR_PROFILE;
  }

  mode_status = BNO055_SetOprMode(hbno, hbno->mode);
  return (status != BNO055_OK) ? status : mode_status;
}

/**
  * @brief  Write a calibration profile at runtime: CONFIGMODE -> offsets/radius -> hbno->mode (3.11.5).
  */
BNO055_Status_t BNO055_WriteCalibProfile(BNO055_HandleTypeDef *hbno, const uint8_t *profile)
{
  BNO055_Status_t status;
  BNO055_Status_t mode_status;

  if ((hbno == NULL) || (profile == NULL))
  {
    return BNO055_ERR_PARAM;
  }
  if (!BNO055_IsProfileValid(profile))
  {
    return BNO055_ERR_PROFILE;
  }

  status = BNO055_SetOprMode(hbno, BNO055_OPR_MODE_CONFIG);
  if (status == BNO055_OK)
  {
    status = BNO055_WriteProfileRegs(hbno, profile);
  }

  mode_status = BNO055_SetOprMode(hbno, hbno->mode);
  return (status != BNO055_OK) ? status : mode_status;
}
