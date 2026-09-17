#pragma once

#include <stdint.h>

/* Safety first: set to 1 only after dry-run calibration without propellers. */
#define ACTUATORS_ENABLED          0
#define NAV_TIMEOUT_MS             300U
#define HBT_TIMEOUT_MS             500U
#define RC_TIMEOUT_MS              250U
#define IMU_TIMEOUT_MS             100U
#define CONTROL_PERIOD_MS          10U
#define TELEMETRY_PERIOD_MS        100U
#define MAX_PITCH_DEG              35.0f
#define MAX_ROLL_DEG               35.0f

/* Standard one-direction ESC. Change only after verifying the actual ESC protocol. */
#define ESC_STOP_US                1000U
#define ESC_MAX_US                 2000U
#define SERVO_MIN_US               1000U
#define SERVO_CENTER_US            1500U
#define SERVO_MAX_US               2000U

/* iBUS: channels are 1000..2000 approximately, channel indexes start at zero. */
#define RC_CH_THROTTLE             2U
#define RC_CH_YAW                  3U
#define RC_CH_MODE                 4U
#define RC_CH_ARM                  5U
#define RC_MODE_AUTO_THRESHOLD     1600U
#define RC_ARM_THRESHOLD           1800U
