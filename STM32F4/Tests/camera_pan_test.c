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

    /* A 30 deg jump (a person): the speed never changes more than ACCEL per tick nor passes MAX_RATE,
       and it settles on the target, overshooting under 1 deg. */
    CameraPan_Init(&pan);
    float v_before = 0.0f, peak = 0.0f;
    for (int i = 0; i < 200; i++) {
        float before = pan.angle_deg;
        CameraPan_Step(&pan, 30.0f, DT);
        float v = (pan.angle_deg - before) / DT;
        assert(fabsf(v - v_before) <= CAM_PAN_ACCEL_DPS2 * DT + 1e-3f);
        assert(fabsf(v) <= CAM_PAN_MAX_RATE_DPS + 1e-3f);
        peak = fmaxf(peak, pan.angle_deg);
        v_before = v;
    }
    close_to(pan.angle_deg, 30.0f, 0.05f);
    assert(peak <= 31.0f);

    /* The Pi's 15 deg/s sweep in 0.75 deg steps every 50 ms (20 Hz): once caught up, every 10 ms tick
       moves the same 0.15 deg, no tick standing still and none jumping: the 20 Hz jerk is gone. */
    CameraPan_Init(&pan);
    float target = 0.0f, smallest = 1e9f, largest = 0.0f;
    for (int i = 0; i < 400; i++) {
        if (i % 5 == 0) target = 15.0f * (float)i * DT;
        float before = pan.angle_deg;
        CameraPan_Step(&pan, target, DT);
        if (i >= 150) {                                   /* after 1.5 s */
            float step = pan.angle_deg - before;
            smallest = fminf(smallest, step);
            largest = fmaxf(largest, step);
        }
    }
    assert(smallest >= 0.15f * 0.7f && largest <= 0.15f * 1.3f);
    close_to(pan.angle_deg, target, 1.0f);                /* on the targets, not a steady lag behind */

    /* The targets stop: so does the servo, past the last one by under 3 deg, and it comes back to it. */
    float stop = pan.angle_deg + 0.75f;
    for (int i = 0; i < 300; i++) CameraPan_Step(&pan, stop, DT);
    close_to(pan.angle_deg, stop, 0.05f);
    CameraPan_Init(&pan);
    for (int i = 0; i < 200; i++) CameraPan_Step(&pan, 0.75f * (float)(i / 5), DT);   /* 15 deg/s for 2 s */
    float last_target = 0.75f * (float)(199 / 5), over = 0.0f;
    for (int i = 0; i < 300; i++) over = fmaxf(over, CameraPan_Step(&pan, last_target, DT) - last_target);
    assert(over < 3.0f);
    close_to(pan.angle_deg, last_target, 0.05f);

    /* A stalled loop (a 10 s tick) moves it no more than a 0.1 s one could. */
    CameraPan_Init(&pan);
    assert(fabsf(CameraPan_Step(&pan, 5.0f, 10.0f)) <= CAM_PAN_ACCEL_DPS2 * 0.1f * 0.1f + 1e-3f);

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
