#include "imu.h"
#include "bno055.h"
#include "bno055_calib_profile.h"
#include "i2c.h"
#include "main.h"

#if BNO055_CALIB_PROFILE_VALID
static const uint8_t imu_calib_profile[BNO055_CALIB_PROFILE_SIZE] = BNO055_CALIB_PROFILE_DATA;
#endif

/* Kept non-static and under these exact names: Tools/CubeMonitor/BNO055_Flow.json
   plots them live, and Tools/bno055_dump_calib.ps1 resolves the last two out of
   the ELF to read a finished calibration off a running target. The handle itself
   is exposed for the same reason - the dashboard shows ext_crystal_active and
   sys_err from it. */
BNO055_HandleTypeDef hbno055;
BNO055_Status_t      bno055_init_status = BNO055_ERR_PARAM;
BNO055_Status_t      bno055_read_status = BNO055_ERR_PARAM;
BNO055_Euler_t       bno055_euler;
BNO055_CalibStatus_t bno055_calib;
uint8_t              bno055_calib_captured[BNO055_CALIB_PROFILE_SIZE];
uint8_t              bno055_calib_captured_valid;

bool IMU_Init(void) {
    hbno055.hi2c = &hi2c1;
    hbno055.address = BNO055_I2C_ADDR_COM3_HIGH; /* ADR floating -> 0x29 */
    hbno055.rst_port = BNO055_RST_GPIO_Port;
    hbno055.rst_pin = BNO055_RST_Pin;
    hbno055.mode = BNO055_OPR_MODE_NDOF;
    hbno055.use_ext_crystal = true; /* 32.768 kHz crystal on the GY-BNO055 */
#if BNO055_CALIB_PROFILE_VALID
    hbno055.calib_profile = imu_calib_profile; /* shared through git */
#else
    hbno055.calib_profile = NULL;
#endif
    bno055_init_status = BNO055_Init(&hbno055);
    return bno055_init_status == BNO055_OK;
}

bool IMU_Read(bno055_euler_t *out) {
    if (bno055_init_status != BNO055_OK) {
        out->valid = false;
        return false;
    }

    bno055_read_status = BNO055_ReadEuler(&hbno055, &bno055_euler);
    if (bno055_read_status != BNO055_OK) {
        out->valid = false;
        return false;
    }

    out->heading_deg = bno055_euler.yaw;
    out->roll_deg = bno055_euler.roll;
    out->pitch_deg = bno055_euler.pitch;

    if (BNO055_ReadCalibStatus(&hbno055, &bno055_calib) == BNO055_OK) {
        out->calibration = bno055_calib.sys;

        /* Snapshot the offsets the first time the sensor reports a complete
           calibration, so the operator can dump and commit a new profile. */
        if ((bno055_calib_captured_valid == 0U) &&
            (bno055_calib.sys == 3U) && (bno055_calib.gyr == 3U) &&
            (bno055_calib.acc == 3U) && (bno055_calib.mag == 3U)) {
            if (BNO055_ReadCalibProfile(&hbno055, bno055_calib_captured) == BNO055_OK) {
                bno055_calib_captured_valid = 1U;
            }
        }
    }

    out->valid = true;
    return true;
}
