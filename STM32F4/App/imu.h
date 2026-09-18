/**
  ******************************************************************************
  * @file    imu.h
  * @brief   Application-level IMU layer on top of the BNO055 driver.
  *
  *          Keeps the flat bno055_euler_t the control loop was written
  *          against, so control.c / main.c stay independent of the driver's
  *          handle-based API and of the sensor's own axis naming.
  ******************************************************************************
  */

#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

/**
  * @brief Orientation in degrees, as consumed by the control loop.
  */
typedef struct {
    float heading_deg;   /* 0 .. 360 */
    float roll_deg;      /* -90 .. +90 */
    float pitch_deg;     /* -180 .. +180 */
    uint8_t calibration; /* system calibration, 0 = none .. 3 = full */
    bool valid;          /* last read succeeded */
} bno055_euler_t;

/**
  * @brief Reset, configure and start the BNO055 in NDOF fusion mode.
  * @retval true on success; false leaves the sensor unusable and every
  *         IMU_Read() will report invalid.
  */
bool IMU_Init(void);

/**
  * @brief Read fused orientation. Call at up to 100 Hz (fusion output rate).
  * @retval true when out holds fresh angles; false sets out->valid to false.
  */
bool IMU_Read(bno055_euler_t *out);

#endif /* IMU_H */
