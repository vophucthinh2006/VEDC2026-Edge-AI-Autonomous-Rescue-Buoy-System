#include "camera_pan.h"
#include "app_config.h"
#include <math.h>

void CameraPan_Init(camera_pan_t *pan) { pan->angle_deg = 0.0f; }

float CameraPan_Step(camera_pan_t *pan, float target_deg, float dt_s) {
    float error = target_deg - pan->angle_deg;
    float rate = fminf(fabsf(error) * CAM_PAN_SMOOTH_KP, CAM_PAN_MAX_RATE_DPS);
    float step = fminf(rate * dt_s, fabsf(error));
    pan->angle_deg += error > 0.0f ? step : -step;
    return pan->angle_deg;
}
