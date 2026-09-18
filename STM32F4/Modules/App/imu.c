#include "imu.h"
#include "bno055.h"
#include "bno055_calib_profile.h"
#include "i2c.h"
#include "main.h"

static BNO055_HandleTypeDef hbno055;
static bool imu_ready;

#if BNO055_CALIB_PROFILE_VALID
static const uint8_t imu_calib_profile[BNO055_CALIB_PROFILE_SIZE] = BNO055_CALIB_PROFILE_DATA;
#endif

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
    imu_ready = (BNO055_Init(&hbno055) == BNO055_OK);
    return imu_ready;
}

bool IMU_Read(bno055_euler_t *out) {
    BNO055_Euler_t euler;
    BNO055_CalibStatus_t calib;

    if (!imu_ready || BNO055_ReadEuler(&hbno055, &euler) != BNO055_OK) {
        out->valid = false;
        return false;
    }

    out->heading_deg = euler.yaw;
    out->roll_deg = euler.roll;
    out->pitch_deg = euler.pitch;
    if (BNO055_ReadCalibStatus(&hbno055, &calib) == BNO055_OK) {
        out->calibration = calib.sys;
    }
    out->valid = true;
    return true;
}
