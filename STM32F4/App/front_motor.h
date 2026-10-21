#pragma once

/* The bidirectional front motor (ZTW Shark, TIM2_CH2 PA1) in AUTO: it pushes forward with the two
 * rear motors, and it brakes. The rear ESCs turn one way only, so on their own the boat coasts on for
 * about a metre after the Pi slows it or stops it; the front motor turned backwards for a moment
 * takes that speed off.
 *
 *   driving at throttle t          front = t * FRONT_THRUST_GAIN
 *   throttle drops by >= FRONT_BRAKE_MIN_DROP (slower, speed 0, STOP, Pi silent, dashboard STOP)
 *                                  front = -FRONT_BRAKE_LEVEL for (drop * FRONT_BRAKE_MS_PER_THROTTLE),
 *                                  at most FRONT_BRAKE_MAX_MS, plus ESC_FRONT_REVERSE_DELAY_MS for the
 *                                  ESC to rest at neutral before it turns the other way
 *   speeding up again              the brake ends at once
 *
 * The drop is measured from the throttle held for a whole FRONT_BRAKE_HOLD_MS (the floor), not from
 * the last command: the Pi flickering between STOP and a creep, or between approach and stand-off,
 * reversed the front motor every 100 ms, and a blip of throttle gives the boat no speed to brake off.
 *
 * Not used when the boat is overturned or its IMU is gone: every motor stops then (App/control.c).
 * Plain C, no HAL: host test in Tests/. */

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float last_throttle;       /* rear throttle of the last call, 0..1 */
    float floor;               /* least throttle of the last whole hold window: what the boat has picked up */
    float window_min;          /* least throttle since window_start_ms */
    uint32_t window_start_ms;
    uint32_t brake_until_ms;
    bool braking;
} front_motor_t;

void FrontMotor_Init(front_motor_t *front);

/* Front command (-1..1, positive pushes the boat forward) for the rear throttle the control loop
 * applies now (0 when it stops the boat). */
float FrontMotor_Command(front_motor_t *front, float throttle, uint32_t now_ms);

/* Every motor off at once, no braking (overturned, IMU lost, disarmed). */
void FrontMotor_Reset(front_motor_t *front);
