#pragma once

/* Glides the camera servo to the Pi's CAM angle instead of jumping there: the motion the
   simulator gives its camera joint (sim/models/vedc_buoy/model.sdf, JointPositionController:
   speed = p_gain * error, capped at cmd_max). Pure C, no HAL: host tests in Tests/. */
typedef struct {
    float angle_deg;    /* where the servo is being driven, from the bow, positive right */
} camera_pan_t;

/* Starts at 0 deg, where Actuators_Init leaves the servo. */
void CameraPan_Init(camera_pan_t *pan);
/* One step of dt_s toward target_deg: speed CAM_PAN_SMOOTH_KP * error, at most
   CAM_PAN_MAX_RATE_DPS, never past the target. Returns the new angle. */
float CameraPan_Step(camera_pan_t *pan, float target_deg, float dt_s);
