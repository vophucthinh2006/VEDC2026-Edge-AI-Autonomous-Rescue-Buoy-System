#pragma once

#include <stdbool.h>

typedef struct {
    float integral;     /* degree-seconds */
    float error_deg;    /* last wrapped error, target - heading */
    float rate_dps;     /* low-passed turn rate used by the D term */
    bool rate_primed;   /* rate_dps holds a sample since the last reset */
    float p, i, d;      /* last terms, for the debugger */
} heading_controller_t;

float Heading_Wrap360(float angle_deg);
float Heading_Error(float target_deg, float current_deg);
void HeadingController_Init(heading_controller_t *controller);
void HeadingController_Reset(heading_controller_t *controller);
/* PID turn command in -1..1, positive turns the bow right. yaw_rate_dps is
   positive clockwise, like the heading. */
float HeadingController_Update(heading_controller_t *controller,
                               float target_deg,
                               float current_deg,
                               float yaw_rate_dps,
                               float dt_s);
/* Turn command u onto the boat: steer = u * HEADING_RUDDER_GAIN, the two rear
   motors split u * HEADING_DIFF_GAIN around throttle, capped so neither goes
   below 0 or above 1. Positive u with HEADING_DIFF_DIR +1 speeds up the left. */
void Heading_Mix(float u, float throttle, float *rear_left, float *rear_right, float *steer);
