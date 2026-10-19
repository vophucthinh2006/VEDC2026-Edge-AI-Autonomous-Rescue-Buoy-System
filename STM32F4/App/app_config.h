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
#define ARM_REQUIRES_IMU           1
/* PE4 (CubeMX label ESTOP) is the victim's confirm button, "supplies taken", not an E-stop:
   App/confirm_button.c counts presses for the Pi, nothing stops. Contact to GND against the
   internal pull-up: pressed reads low. The boat has no E-stop input; the RC (SwD) disarms. */
#define CONFIRM_DEBOUNCE_MS        50U
#if ACTUATORS_ENABLED && !ARM_REQUIRES_IMU
#warning "Arming without the IMU: no overturn stop. Bench only."
#endif
/* Boat on its stand only: 1 arms and runs AUTO with a failed or stale BNO055. The heading then
   stays where the IMU last read it and the turn rate counts as 0, so the PID turns the boat by
   the camera's bearing to the person (the Pi's target is that heading plus the bearing). No
   overturn stop. Set back to 0 before any water run. */
#define BENCH_IGNORE_IMU           0
#if ACTUATORS_ENABLED && BENCH_IGNORE_IMU
#warning "AUTO runs without a fresh IMU: boat on the stand only."
#endif
/* Boat on its stand only, no RC transmitter: 1 makes the PE4 button the arm switch, in AUTO.
   Off at boot; one press arms, the next disarms and stops the motors at once. Armed, the motors
   still run only while the Pi sends NAV and HBT. The RC is not read for arming. Set back to 0
   before any water run. */
#define BENCH_PE4_ARM              0
#if ACTUATORS_ENABLED && BENCH_PE4_ARM
#warning "PE4 arms the boat in AUTO instead of the RC: boat on the stand only."
#endif
#define MAX_PITCH_DEG              35.0f
#define MAX_ROLL_DEG               35.0f

/* Heading reference. East magnetic declination and a clockwise sensor
   mounting correction are positive. Calibrate both values on the finished
   boat before a water run. */
#define IMU_MOUNTING_OFFSET_DEG    0.0f
#define MAGNETIC_DECLINATION_DEG   0.0f
/* Gyro Z to a clockwise turn rate. BNO055 flat, Z up: gyro Z is positive
   anticlockwise, hence -1. Check on the desk: turn the sensor clockwise
   (seen from above), yaw_rate_dps must read positive. */
#define IMU_YAW_RATE_SIGN          (-1.0f)

/* Heading PID (AUTO and RC heading hold). Output u in -1..1, positive turns
   the bow right. u = Kp*e + Ki*integral(e) - Kd*yaw_rate: the D term works on
   the gyro, so a target jump gives no kick and yaw noise is not differentiated.
   Starting values, tune on the stand then on water: P first, then D until the
   overshoot is gone, then a little I for a steady drift. */
#define HEADING_KP                 0.020f  /* per degree: 20 deg error gives u = 0.4 */
#define HEADING_KI                 0.002f  /* per degree-second */
#define HEADING_KD                 0.008f  /* per deg/s: 30 deg/s turn gives 0.24 of braking */
#define HEADING_DEADBAND_DEG       2.0f    /* P rests inside, I is held, D still damps */
#define HEADING_INTEGRAL_LIMIT     100.0f  /* degree-seconds: Ki * limit = 0.2 of u */
#define HEADING_RATE_LPF_HZ        5.0f    /* low-pass on the gyro rate for the D term */
/* Mixing u onto the boat: all rudders get u * RUDDER_GAIN, the rear motors
   split u * DIFF_GAIN around their common throttle (capped so neither goes
   below zero). DIFF_DIR +1: positive u speeds the LEFT motor up, which pushes
   the bow right. Flip it if the stand test shows the motors helping the wrong way. */
#define HEADING_RUDDER_GAIN        1.0f
#define HEADING_DIFF_GAIN          0.30f
#define HEADING_DIFF_DIR           (+1)
/* RC heading hold in MANUAL: right stick forward with the steering centred
   locks the current heading and the PID holds it. Moving the steering,
   reversing or dropping the throttle releases it. The lock waits for the
   turn to settle below LOCK_RATE, or LOCK_TIMEOUT at most. */
#define RC_HEADING_HOLD_ENABLED    1
#define RC_HOLD_STEER_BAND_US      60U     /* CH1 within 1500 +/- this counts as let go; wider than the stick deadband for trim and sub-trim */
#define RC_HOLD_LOCK_RATE_DPS      10.0f
#define RC_HOLD_LOCK_TIMEOUT_MS    1000U

/* Rear ESCs: unbranded 30 A, one direction, PWM 50 Hz (TIM2_CH1 PA5, TIM2_CH4 PA3). Their
   throttle range is fixed (no calibration): stepped on the bench (2026-10-18) they stay still up to
   1490 us and turn from 1500 us; 1500..2000 us is the range settled on after the motor phase swap.
   Any rear demand above zero is spread over START..REAR_MAX. */
#define ESC_STOP_US                1000U
#define ESC_REAR_START_US          1500U
#define ESC_REAR_MAX_US            2000U
#define ESC_MAX_US                 2000U
/* Front ESC (ZTW Shark G2 50A) is bidirectional: 1500 us is stop, 1000 and 2000 us
   are full speed either way. PWM 50 Hz on TIM2_CH2, PA1. */
#define ESC_FRONT_NEUTRAL_US       1500U
/* Pulse direction for a forward push. -1: above 1500 us the front prop pushes against the rear
   ones (bench, 2026-10-18), so forward is below neutral and reverse (the brake) above it. */
#define ESC_FRONT_DIR              (-1)
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
   instead of the control loop. ESC output is capped and time-limited. The first three
   can be set from the build: the HwTest preset (motor_test.bat) does. */
#ifndef HW_TEST_ENABLED
#define HW_TEST_ENABLED            0
#endif
#ifndef HW_TEST_ESC_MAX_US
#define HW_TEST_ESC_MAX_US         1500U   /* props stay on: half throttle */
#endif
#ifndef HW_TEST_ESC_FRONT_SPAN_US
#define HW_TEST_ESC_FRONT_SPAN_US  150U    /* front ESC: neutral +/- this, about 30 % each way; water-cooled, keep runs short */
#endif
#define HW_TEST_ESC_MAX_RUN_MS     60000U  /* motors on a stand; a lost debugger still stops them */
#define HW_TEST_ESC_ARM_AT_BOOT    1       /* stop signal from boot (1000 us rear, neutral front) so the ESCs arm */
#define HW_TEST_SWEEP_US           350U    /* rudder swing each side of centre */

/* Camera pan servo (SERVO_CAMERA, TIM3_CH1) follows the CAM packet from the Pi.
   Pulse = centre + dir * us_per_deg * angle. Flip DIR if the camera turns away from the person. */
#define CAM_PAN_US_PER_DEG         11.0f   /* measured on the boat: 1005 us turned it 45 deg from 1500 */
#define CAM_PAN_DIR                (-1)
#define CAM_PAN_LIMIT_DEG          80.0f
/* App/camera_pan.c glides the servo to the CAM angle: speed = KP * error, at most MAX_RATE.
   The simulator's joint: 20 /s and 573 deg/s; softer here for the real servo and camera head.
   KP 10 /s settles a step in about 0.3 s and trails a 30 deg/s sweep by 3 deg. */
#define CAM_PAN_SMOOTH_KP          10.0f   /* per second */
#define CAM_PAN_MAX_RATE_DPS       180.0f
/* The camera servo needs more than 1000..2000 us for +-80 deg; the rudders keep SERVO_MIN/MAX_US. */
#define CAM_SERVO_MIN_US           600U
#define CAM_SERVO_MAX_US           2400U

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
