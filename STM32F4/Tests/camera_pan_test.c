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

    /* Pan targets past +-90 deg stop at the limit, and its pulses stay in the limits' pulses. */
    CameraPan_Init(&pan);
    for (int i = 0; i < 300; i++) CameraPan_Step(&pan, 500.0f, DT);
    close_to(pan.angle_deg, CAM_PAN_LIMIT_DEG, 0.0001f);
    uint32_t pan_a = CameraPan_Us(-CAM_PAN_LIMIT_DEG), pan_b = CameraPan_Us(CAM_PAN_LIMIT_DEG);
    uint32_t pan_lo = pan_a < pan_b ? pan_a : pan_b, pan_hi = pan_a < pan_b ? pan_b : pan_a;
    assert(pan_lo >= CAM_SERVO_MIN_US && pan_hi <= CAM_SERVO_MAX_US);
    assert(CameraPan_Us(0.0f) == CAM_PAN_CENTER_US);
    for (float a = -400.0f; a <= 400.0f; a += 0.5f) {
        uint32_t us = CameraPan_Us(a);
        assert(us >= pan_lo && us <= pan_hi);
    }

    /* Tilt at power-up: from the frame (0) to the start angle, never faster than its rate, no overshoot. */
    camera_pan_t tilt;
    CameraTilt_Init(&tilt);
    close_to(tilt.angle_deg, 0.0f, 0.0f);
    for (int i = 0; i < 300; i++) {
        float before = tilt.angle_deg;
        CameraTilt_Step(&tilt, CAM_TILT_START_DEG, DT);
        assert(tilt.angle_deg - before <= CAM_TILT_MAX_RATE_DPS * DT + 1e-5f);
        assert(tilt.angle_deg >= before && tilt.angle_deg <= CAM_TILT_START_DEG);
    }
    close_to(tilt.angle_deg, CAM_TILT_START_DEG, 0.01f);

    /* Tilt targets outside 0..60 stop at the limits, at most the tilt rate per tick. */
    for (int i = 0; i < 300; i++) {
        float before = tilt.angle_deg;
        CameraTilt_Step(&tilt, 200.0f, DT);
        assert(fabsf(tilt.angle_deg - before) <= CAM_TILT_MAX_RATE_DPS * DT + 1e-5f);
    }
    close_to(tilt.angle_deg, CAM_TILT_MAX_DEG, 0.0001f);
    for (int i = 0; i < 300; i++) CameraTilt_Step(&tilt, -45.0f, DT);
    close_to(tilt.angle_deg, CAM_TILT_MIN_DEG, 0.0001f);

    /* Tilt pulses never leave those of 0..60 deg, whatever angle reaches them. */
    uint32_t tilt_a = CameraTilt_Us(CAM_TILT_MIN_DEG), tilt_b = CameraTilt_Us(CAM_TILT_MAX_DEG);
    uint32_t tilt_lo = tilt_a < tilt_b ? tilt_a : tilt_b, tilt_hi = tilt_a < tilt_b ? tilt_b : tilt_a;
    assert(tilt_a == CAM_TILT_ZERO_US && tilt_hi - tilt_lo == (uint32_t)(CAM_TILT_US_PER_DEG * CAM_TILT_MAX_DEG));
    assert(tilt_lo >= CAM_SERVO_MIN_US && tilt_hi <= CAM_SERVO_MAX_US);
    for (float a = -400.0f; a <= 400.0f; a += 0.5f) {
        uint32_t us = CameraTilt_Us(a);
        assert(us >= tilt_lo && us <= tilt_hi);
    }

    printf("camera_pan_test: PASS\n");
    return 0;
}
