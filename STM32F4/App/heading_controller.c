#include "heading_controller.h"
#include "app_config.h"
#include <math.h>

#define OUTPUT_LIMIT 1.0f

static float clampf(float value, float low, float high) {
    return value < low ? low : (value > high ? high : value);
}

float Heading_Wrap360(float angle_deg) {
    float wrapped = fmodf(angle_deg, 360.0f);
    return wrapped < 0.0f ? wrapped + 360.0f : wrapped;
}

float Heading_Error(float target_deg, float current_deg) {
    return fmodf(target_deg - current_deg + 540.0f, 360.0f) - 180.0f;
}

void HeadingController_Init(heading_controller_t *controller) {
    HeadingController_Reset(controller);
}

void HeadingController_Reset(heading_controller_t *controller) {
    controller->integral = 0.0f;
    controller->error_deg = 0.0f;
    controller->rate_dps = 0.0f;
    controller->rate_primed = false;
    controller->p = controller->i = controller->d = 0.0f;
}

float HeadingController_Update(heading_controller_t *controller,
                               float target_deg,
                               float current_deg,
                               float yaw_rate_dps,
                               float dt_s) {
    const float error = Heading_Error(target_deg, current_deg);
    controller->error_deg = error;

    /* First-order low-pass on the gyro, primed with the first sample so the
       D term does not ramp up from zero after a reset. */
    if (!controller->rate_primed) {
        controller->rate_dps = yaw_rate_dps;
        controller->rate_primed = true;
    } else {
        const float tau = 1.0f / (2.0f * 3.14159265f * HEADING_RATE_LPF_HZ);
        controller->rate_dps += (dt_s / (tau + dt_s)) * (yaw_rate_dps - controller->rate_dps);
    }
    controller->d = -HEADING_KD * controller->rate_dps;

    /* Inside the deadband the P term rests but the integral is held, not
       cleared: it carries the steady trim that offsets unequal motors. */
    if (fabsf(error) <= HEADING_DEADBAND_DEG) {
        controller->p = 0.0f;
        controller->i = HEADING_KI * controller->integral;
        return clampf(controller->i + controller->d, -OUTPUT_LIMIT, OUTPUT_LIMIT);
    }

    const float candidate = clampf(controller->integral + error * dt_s,
                                   -HEADING_INTEGRAL_LIMIT,
                                   HEADING_INTEGRAL_LIMIT);
    controller->p = HEADING_KP * error;
    const float raw = controller->p + HEADING_KI * candidate + controller->d;
    const float output = clampf(raw, -OUTPUT_LIMIT, OUTPUT_LIMIT);

    /* Integrate only when unsaturated, or when the error would pull a
       saturated output back toward the usable range. */
    if (raw == output ||
        (output >= OUTPUT_LIMIT && error < 0.0f) ||
        (output <= -OUTPUT_LIMIT && error > 0.0f)) {
        controller->integral = candidate;
    }
    controller->i = HEADING_KI * controller->integral;
    return output;
}

void Heading_Mix(float u, float throttle, float *rear_left, float *rear_right, float *steer) {
    throttle = clampf(throttle, 0.0f, 1.0f);
    const float room = fminf(throttle, 1.0f - throttle);
    const float diff = clampf(u * HEADING_DIFF_GAIN * (float)HEADING_DIFF_DIR, -room, room);
    *rear_left = throttle + diff;
    *rear_right = throttle - diff;
    *steer = clampf(u * HEADING_RUDDER_GAIN, -1.0f, 1.0f);
}
