#include "camera_pan.h"
#include "app_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define DT 0.01f   /* CONTROL_PERIOD_MS */

static void close_to(float actual, float expected, float tolerance) {
    assert(fabsf(actual - expected) <= tolerance);
}

int main(void) {
    camera_pan_t pan;

    /* A 10 deg step, under the rate cap: the first tick moves KP * 10 * dt. */
    CameraPan_Init(&pan);
    close_to(CameraPan_Step(&pan, 10.0f, DT), CAM_PAN_SMOOTH_KP * 10.0f * DT, 0.001f);

    /* A 30 deg step: no jump, and it gets there without overshoot. */
    CameraPan_Init(&pan);
    float last = pan.angle_deg;
    for (int i = 0; i < 100; i++) {
        float angle = CameraPan_Step(&pan, 30.0f, DT);
        assert(angle >= last && angle <= 30.0f);
        last = angle;
    }
    close_to(pan.angle_deg, 30.0f, 0.1f);

    /* A big jump is capped at the top rate. */
    CameraPan_Init(&pan);
    close_to(CameraPan_Step(&pan, -160.0f, DT), -CAM_PAN_MAX_RATE_DPS * DT, 0.001f);

    /* A late tick never carries it past the target. */
    CameraPan_Init(&pan);
    close_to(CameraPan_Step(&pan, 5.0f, 10.0f), 5.0f, 0.0001f);

    /* Commands at 20 Hz along a 30 deg/s sweep: followed smoothly, about rate / KP behind. */
    CameraPan_Init(&pan);
    float target = 0.0f, biggest_tick = 0.0f;
    for (int i = 0; i < 300; i++) {
        if (i % 5 == 0) target = 30.0f * (float)i * DT;   /* a new CAM packet every 50 ms */
        float before = pan.angle_deg;
        CameraPan_Step(&pan, target, DT);
        biggest_tick = fmaxf(biggest_tick, fabsf(pan.angle_deg - before));
    }
    assert(biggest_tick < 1.0f);                                 /* < 1 deg per 10 ms tick: no jumps */
    close_to(target - pan.angle_deg, 30.0f / CAM_PAN_SMOOTH_KP, 1.5f);   /* plus up to one 1.5 deg packet step */

    printf("camera_pan_test: PASS\n");
    return 0;
}
