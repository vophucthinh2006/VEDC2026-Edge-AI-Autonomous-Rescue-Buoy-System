#include "front_motor.h"
#include "app_config.h"
#include <string.h>

void FrontMotor_Init(front_motor_t *front)
{
    memset(front, 0, sizeof(*front));
}

void FrontMotor_Reset(front_motor_t *front)
{
    front->last_throttle = 0.0f;
    front->floor = 0.0f;
    front->window_min = 0.0f;
    front->braking = false;
}

float FrontMotor_Command(front_motor_t *front, float throttle, uint32_t now_ms)
{
    if (throttle < 0.0f) throttle = 0.0f;
    /* Only the throttle the boat has held long enough to pick up speed is braked off: a blip of
       throttle between two stops, or the Pi flickering between two speeds, gives none. */
    float held = front->floor < front->last_throttle ? front->floor : front->last_throttle;
    float drop = held - throttle;
    if (drop >= FRONT_BRAKE_MIN_DROP) {
        /* A bigger cut has more speed to take off: brake for longer, within a cap. */
        float ms = drop * (float)FRONT_BRAKE_MS_PER_THROTTLE;
        if (ms > (float)FRONT_BRAKE_MAX_MS) ms = (float)FRONT_BRAKE_MAX_MS;
        front->brake_until_ms = now_ms + ESC_FRONT_REVERSE_DELAY_MS + (uint32_t)ms;
        front->braking = true;
    } else if (throttle > front->last_throttle) {
        front->braking = false;   /* speeding up again: no more braking */
    }
    front->last_throttle = throttle;
    /* The floor falls with the throttle at once and rises only to the least throttle of a whole
       FRONT_BRAKE_HOLD_MS window. */
    if (throttle < front->floor) front->floor = throttle;
    if (throttle < front->window_min) front->window_min = throttle;
    if ((uint32_t)(now_ms - front->window_start_ms) >= FRONT_BRAKE_HOLD_MS) {
        front->floor = front->window_min;
        front->window_min = throttle;
        front->window_start_ms = now_ms;
    }
    if (front->braking) {
        if ((int32_t)(now_ms - front->brake_until_ms) < 0) return -FRONT_BRAKE_LEVEL;
        front->braking = false;
    }
    return throttle * FRONT_THRUST_GAIN;
}
