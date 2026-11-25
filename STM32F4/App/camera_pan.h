#pragma once

#include <stdint.h>

/* Glides the camera gimbal (pan on PA6, tilt on PD12) to its target instead of jumping there: the
   motion the simulator gives its camera joint (sim/models/vedc_buoy/model.sdf, JointPositionController:
   speed = p_gain * error, capped at cmd_max). Pure C, no HAL: host tests in Tests/. */
typedef struct {
    float angle_deg;    /* where the servo is being driven: pan from the bow, positive right; tilt 0 = resting on the frame */
    /* Pan only (CameraPan_Step); sim/tools/f407_board_sim.py mirrors this layout for ctypes. */
    float velocity_dps;     /* the servo's commanded speed, changed at most CAM_PAN_ACCEL_DPS2 */
    float last_target_deg;  /* the last target received */
    float target_rate_dps;  /* how fast the targets move (the Pi's sweep or the person), 0 when they stop */
    float since_target_s;   /* since the last target changed */
} camera_pan_t;

/* Pan starts at 0 deg, where Actuators_Init leaves the servo. */
void CameraPan_Init(camera_pan_t *pan);
/* One step of dt_s toward target_deg, clamped to +-CAM_PAN_LIMIT_DEG. The Pi's targets come in steps (20 a
   second); followed with a plain P glide the servo jerked at 20 Hz. So the step learns how fast the targets
   move, runs at that speed between them toward where the next one will be (at most CAM_PAN_FF_STALE_S
   ahead), and corrects with CAM_PAN_SMOOTH_KP; speed at most CAM_PAN_MAX_RATE_DPS, changed at most
   CAM_PAN_ACCEL_DPS2. Returns the new angle. */
float CameraPan_Step(camera_pan_t *pan, float target_deg, float dt_s);
/* Pan angle to a pulse, never outside the pulses of +-CAM_PAN_LIMIT_DEG. */
uint32_t CameraPan_Us(float angle_deg);

/* Tilt starts at 0 deg, the platform resting on the frame: unpowered, that is where it sits. */
void CameraTilt_Init(camera_pan_t *tilt);
/* As CameraPan_Step, target clamped to CAM_TILT_MIN_DEG..CAM_TILT_MAX_DEG, at most CAM_TILT_MAX_RATE_DPS. */
float CameraTilt_Step(camera_pan_t *tilt, float target_deg, float dt_s);
/* Tilt angle to a pulse, never outside the pulses of CAM_TILT_MIN_DEG..CAM_TILT_MAX_DEG. */
uint32_t CameraTilt_Us(float angle_deg);
