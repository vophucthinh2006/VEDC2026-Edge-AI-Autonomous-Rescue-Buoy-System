#pragma once

#include <stdint.h>

/* Glides the camera gimbal (pan on PA6, tilt on PD12) to its target instead of jumping there: the
   motion the simulator gives its camera joint (sim/models/vedc_buoy/model.sdf, JointPositionController:
   speed = p_gain * error, capped at cmd_max). Pure C, no HAL: host tests in Tests/. */
typedef struct {
    float angle_deg;    /* where the servo is being driven: pan from the bow, positive right; tilt 0 = resting on the frame */
} camera_pan_t;

/* Pan starts at 0 deg, where Actuators_Init leaves the servo. */
void CameraPan_Init(camera_pan_t *pan);
/* One step of dt_s toward target_deg, clamped to +-CAM_PAN_LIMIT_DEG: speed CAM_PAN_SMOOTH_KP * error,
   at most CAM_PAN_MAX_RATE_DPS, never past the target. Returns the new angle. */
float CameraPan_Step(camera_pan_t *pan, float target_deg, float dt_s);
/* Pan angle to a pulse, never outside the pulses of +-CAM_PAN_LIMIT_DEG. */
uint32_t CameraPan_Us(float angle_deg);

/* Tilt starts at 0 deg, the platform resting on the frame: unpowered, that is where it sits. */
void CameraTilt_Init(camera_pan_t *tilt);
/* As CameraPan_Step, target clamped to CAM_TILT_MIN_DEG..CAM_TILT_MAX_DEG, at most CAM_TILT_MAX_RATE_DPS. */
float CameraTilt_Step(camera_pan_t *tilt, float target_deg, float dt_s);
/* Tilt angle to a pulse, never outside the pulses of CAM_TILT_MIN_DEG..CAM_TILT_MAX_DEG. */
uint32_t CameraTilt_Us(float angle_deg);
