#include "camera_pan.h"
#include "app_config.h"
#include <math.h>

static float clampf(float value, float low, float high) { return fminf(fmaxf(value, low), high); }

static float glide(camera_pan_t *axis, float target_deg, float dt_s, float max_rate_dps) {
    float error = target_deg - axis->angle_deg;
    float rate = fminf(fabsf(error) * CAM_PAN_SMOOTH_KP, max_rate_dps);
    float step = fminf(rate * dt_s, fabsf(error));
    axis->angle_deg += error > 0.0f ? step : -step;
    return axis->angle_deg;
}

/* Pulse of angle_deg, kept between the pulses of the two angle limits whatever the sign of dir. */
static uint32_t to_us(float angle_deg, float zero_us, float dir_us_per_deg, float low_deg, float high_deg) {
    float a = zero_us + dir_us_per_deg * low_deg, b = zero_us + dir_us_per_deg * high_deg;
    return (uint32_t)lroundf(clampf(zero_us + dir_us_per_deg * angle_deg, fminf(a, b), fmaxf(a, b)));
}

void CameraPan_Init(camera_pan_t *pan) {
    pan->angle_deg = pan->velocity_dps = pan->last_target_deg = pan->target_rate_dps = 0.0f;
    pan->since_target_s = 1000.0f;   /* no target yet: the first one gives no speed */
}

float CameraPan_Step(camera_pan_t *pan, float target_deg, float dt_s) {
    float target = clampf(target_deg, -CAM_PAN_LIMIT_DEG, CAM_PAN_LIMIT_DEG);
    float dt = fminf(dt_s, 0.1f);    /* a stalled loop: no leap */
    pan->since_target_s += dt;
    if (target != pan->last_target_deg) {
        /* A new target: how fast they move. After a pause (a jump to a person) the first one says nothing. */
        float rate = pan->since_target_s > CAM_PAN_FF_STALE_S ? 0.0f
                   : (target - pan->last_target_deg) / fmaxf(pan->since_target_s, 0.02f);
        pan->target_rate_dps = clampf(0.5f * (pan->target_rate_dps + rate), -CAM_PAN_FF_MAX_DPS, CAM_PAN_FF_MAX_DPS);
        pan->last_target_deg = target;
        pan->since_target_s = 0.0f;
    } else if (pan->since_target_s > CAM_PAN_FF_STALE_S) {
        pan->target_rate_dps = 0.0f;   /* the targets stopped */
    }
    /* Where the target is by now, not where the last packet put it: no sawtooth error between packets. */
    float ahead = clampf(target + pan->target_rate_dps * fminf(pan->since_target_s, CAM_PAN_FF_STALE_S),
                         -CAM_PAN_LIMIT_DEG, CAM_PAN_LIMIT_DEG);
    /* The correction no faster than it can still brake within the distance left: no overshoot. */
    float error = ahead - pan->angle_deg;
    float correction = copysignf(fminf(CAM_PAN_SMOOTH_KP * fabsf(error), sqrtf(2.0f * CAM_PAN_ACCEL_DPS2 * fabsf(error))), error);
    float wanted = clampf(pan->target_rate_dps + correction, -CAM_PAN_MAX_RATE_DPS, CAM_PAN_MAX_RATE_DPS);
    float dv = CAM_PAN_ACCEL_DPS2 * dt;
    pan->velocity_dps += clampf(wanted - pan->velocity_dps, -dv, dv);
    pan->angle_deg += pan->velocity_dps * dt;
    if (fabsf(pan->angle_deg) >= CAM_PAN_LIMIT_DEG) {
        pan->angle_deg = clampf(pan->angle_deg, -CAM_PAN_LIMIT_DEG, CAM_PAN_LIMIT_DEG);
        pan->velocity_dps = 0.0f;
    }
    return pan->angle_deg;
}

uint32_t CameraPan_Us(float angle_deg) {
    return to_us(angle_deg, (float)CAM_PAN_CENTER_US, (float)CAM_PAN_DIR * CAM_PAN_US_PER_DEG, -CAM_PAN_LIMIT_DEG, CAM_PAN_LIMIT_DEG);
}

void CameraTilt_Init(camera_pan_t *tilt) { CameraPan_Init(tilt); }   /* the pan fields stay unused */

float CameraTilt_Step(camera_pan_t *tilt, float target_deg, float dt_s) {
    return glide(tilt, clampf(target_deg, CAM_TILT_MIN_DEG, CAM_TILT_MAX_DEG), dt_s, CAM_TILT_MAX_RATE_DPS);
}

uint32_t CameraTilt_Us(float angle_deg) {
    return to_us(angle_deg, (float)CAM_TILT_ZERO_US, (float)CAM_TILT_DIR * CAM_TILT_US_PER_DEG, CAM_TILT_MIN_DEG, CAM_TILT_MAX_DEG);
}
