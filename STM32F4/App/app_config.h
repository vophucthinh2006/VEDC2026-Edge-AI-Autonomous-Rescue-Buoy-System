#pragma once

#include <stdint.h>

/* Safety first: set to 1 only after dry-run calibration without propellers.
   At 0 the ESCs hold stop while the rudders still follow the commands. */
#define ACTUATORS_ENABLED          1
#define NAV_TIMEOUT_MS             300U
#define HBT_TIMEOUT_MS             500U
#define RC_TIMEOUT_MS              250U
#define IMU_TIMEOUT_MS             100U
#define CONTROL_PERIOD_MS          10U
#define TELEMETRY_PERIOD_MS        100U
/* UART4 link test: 1 replaces the IMU/GPS/SYS telemetry with a counting
   $TST line every TELEMETRY_PERIOD_MS, so the Pi side can be checked alone. */
#define PI_LINK_TEST_ENABLED       0
/* Arming needs fresh BNO055 data. 0 only while the IMU is not wired yet:
   the overturn check is then off too. Set back to 1 before any water run. */
#define ARM_REQUIRES_IMU           0
/* E-stop contact on PE4, internal pull-up. 1 = normally closed, the design:
   pressed or a cut wire reads high and stops. 0 = normally open, bench only:
   pressed reads low and stops, but a cut wire reads as released. */
#define ESTOP_CONTACT_NC           0
#if ACTUATORS_ENABLED && !ESTOP_CONTACT_NC
#warning "E-stop on its NO contact: a cut wire does not stop the motors. Bench only."
#endif
#if ACTUATORS_ENABLED && !ARM_REQUIRES_IMU
#warning "Arming without the IMU: no overturn stop. Bench only."
#endif
#define MAX_PITCH_DEG              35.0f
#define MAX_ROLL_DEG               35.0f

/* Rear ESCs: one-direction BLHeli_S, driven over DShot; the microsecond range
   below is kept as the command scale. */
#define ESC_STOP_US                1000U
#define ESC_MAX_US                 2000U
/* Front ESC (ZTW Shark G2 50A) is bidirectional: 1500 us is stop, 1000 us is
   full reverse. It stays on plain PWM (TIM5_CH2, PA1). */
#define ESC_FRONT_NEUTRAL_US       1500U
#define SERVO_MIN_US               1000U
#define SERVO_CENTER_US            1500U
#define SERVO_MAX_US               2000U
/* Straight-ahead pulse per rudder servo, trimmed on the bench.
   Lower pulse turns the pod right. The left one sits far from 1500 because
   of how its horn is mounted, which leaves it little travel to the right;
   the same goes for the front one. */
#define SERVO_CENTER_FRONT_US      1280U
#define SERVO_CENTER_RIGHT_US      1460U
#define SERVO_CENTER_LEFT_US       1280U
/* Rudder swing at full steer, the same each way so the boat tracks straight
   with the stick centred. The front and left rudders only have about 280 us
   to the right. */
#define RUDDER_SPAN_US             250U
/* Pulse direction for positive steer (bow right). The front pod and the two
   rear pods swing opposite ways, like four-wheel steering. Check on the stand:
   stick right must pull the bow right and push the stern left. */
#define RUDDER_DIR_FRONT           (-1)
#define RUDDER_DIR_REAR            (+1)
#define RUDDER_FRONT_GAIN          1.0f    /* front swing relative to the rear ones */

/* ESC command ramp, both modes: speed-ups take at least 0.5 s from stop to
   full, cuts are immediate. The front ESC rests at neutral this long before
   it spins the other way. */
#define ESC_RAMP_PER_S             2.0f
#define ESC_FRONT_REVERSE_DELAY_MS 200U

/* Bench test (App/hw_test.c): servos, ESCs and buzzer driven from the debugger
   instead of the control loop. ESC output is capped and time-limited. */
#define HW_TEST_ENABLED            0
#define HW_TEST_ESC_MAX_US         1500U   /* props stay on: about DShot 1047, half throttle */
#define HW_TEST_ESC_FRONT_SPAN_US  150U    /* front ESC: neutral +/- this, about 30 % each way; water-cooled, keep runs short */
#define HW_TEST_ESC_MAX_RUN_MS     60000U  /* motors on a stand; a lost debugger still stops them */
#define HW_TEST_ESC_ARM_AT_BOOT    1       /* stop signal from boot (DShot rear, neutral front) so the ESCs arm */
#define HW_TEST_SWEEP_US           350U    /* rudder swing each side of centre */

/* Camera pan servo (SERVO_CAMERA, TIM3_CH1) follows the CAM packet from the Pi.
   Pulse = centre + dir * us_per_deg * angle. Flip DIR if the camera turns away from the person. */
#define CAM_PAN_US_PER_DEG         5.5f    /* 1000..2000 us over about 180 deg */
#define CAM_PAN_DIR                (-1)
#define CAM_PAN_LIMIT_DEG          80.0f

/* iBUS: channels are 1000..2000 approximately, channel indexes start at zero.
   FS-i6 in stick mode 2, SwB and SwD set as the aux sources of CH5 and CH6. */
#define RC_CH_STEER                0U      /* CH1, right stick sideways */
#define RC_CH_SPEED                1U      /* CH2, right stick up/down: forward/reverse */
#define RC_CH_POWER                2U      /* CH3, left stick up/down: power limit */
#define RC_CH_MODE                 4U      /* CH5, SwB */
#define RC_CH_ARM                  5U      /* CH6, SwD */
#define RC_MODE_AUTO_THRESHOLD     1600U
#define RC_ARM_THRESHOLD           1800U
#define RC_POWER_MIN_US            1050U   /* power stick at or below this is zero, and needed to arm */
#define RC_STICK_DEADBAND_US       30U     /* around 1500 on the self-centring sticks */

/* Manual caps, fractions of full ESC output. Start low, raise after water runs. */
#define MANUAL_REAR_MAX            1.00f
#define MANUAL_FRONT_FWD_GAIN      0.0f    /* front motor share when going forward; try 0.5 later */
#define MANUAL_FRONT_REV_MAX       0.30f   /* front motor in reverse, also the brake */

/* LoRa: the boat sends its GPS position to the shore station (App/lora_beacon.c).
   The radio settings MUST match the station (esp32-lora-station/main/main.c). Fit the
   433 MHz antenna on the RA-02 before enabling: transmitting without one can damage the PA. */
#define LORA_ENABLED               1
#define LORA_BUOY_ID               "PHAO-01"   /* name shown on the dashboard; change it if there are several boats */
#define LORA_FREQUENCY_HZ          433000000UL
#define LORA_SF                    9U
#define LORA_BW_HZ                 125000UL
#define LORA_CR                    5U          /* 4/5 */
#define LORA_SYNC_WORD             0xF3U
#define LORA_TX_POWER_DBM          17U         /* PA_BOOST; 17 dBm is the most that needs no PA_DAC */
#define LORA_BEACON_PERIOD_MS      5000U
#define LORA_GPS_STALE_MS          3000U       /* GPS silent for this long: send NO_FIX */
