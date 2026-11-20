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

void CameraPan_Init(camera_pan_t *pan) { pan->angle_deg = 0.0f; }

float CameraPan_Step(camera_pan_t *pan, float target_deg, float dt_s) {
    return glide(pan, clampf(target_deg, -CAM_PAN_LIMIT_DEG, CAM_PAN_LIMIT_DEG), dt_s, CAM_PAN_MAX_RATE_DPS);
}

uint32_t CameraPan_Us(float angle_deg) {
    return to_us(angle_deg, (float)CAM_PAN_CENTER_US, (float)CAM_PAN_DIR * CAM_PAN_US_PER_DEG, -CAM_PAN_LIMIT_DEG, CAM_PAN_LIMIT_DEG);
}

void CameraTilt_Init(camera_pan_t *tilt) { tilt->angle_deg = 0.0f; }

float CameraTilt_Step(camera_pan_t *tilt, float target_deg, float dt_s) {
    return glide(tilt, clampf(target_deg, CAM_TILT_MIN_DEG, CAM_TILT_MAX_DEG), dt_s, CAM_TILT_MAX_RATE_DPS);
}

uint32_t CameraTilt_Us(float angle_deg) {
    return to_us(angle_deg, (float)CAM_TILT_ZERO_US, (float)CAM_TILT_DIR * CAM_TILT_US_PER_DEG, CAM_TILT_MIN_DEG, CAM_TILT_MAX_DEG);
}
