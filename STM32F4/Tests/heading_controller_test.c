#include "heading_controller.h"
#include "app_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define DT 0.01f

static void close_to(float actual, float expected, float tolerance) {
    assert(fabsf(actual - expected) <= tolerance);
}

int main(void) {
    close_to(Heading_Wrap360(361.0f), 1.0f, 0.001f);
    close_to(Heading_Wrap360(-1.0f), 359.0f, 0.001f);
    close_to(Heading_Error(1.0f, 359.0f), 2.0f, 0.001f);
    close_to(Heading_Error(359.0f, 1.0f), -2.0f, 0.001f);

    heading_controller_t c;

    /* P: 10 deg right of the target turns the bow right. */
    HeadingController_Init(&c);
    float u = HeadingController_Update(&c, 10.0f, 0.0f, 0.0f, DT);
    close_to(u, HEADING_KP * 10.0f + HEADING_KI * 10.0f * DT, 0.0005f);
    assert(u > 0.0f && c.error_deg > 0.0f);

    /* Across north: target 350, heading 0 turns left. */
    HeadingController_Reset(&c);
    assert(HeadingController_Update(&c, 350.0f, 0.0f, 0.0f, DT) < 0.0f);

    /* D on the gyro: already turning right at 30 deg/s toward the target,
       the D term brakes; it does not depend on the target at all. */
    HeadingController_Reset(&c);
    u = HeadingController_Update(&c, 10.0f, 0.0f, 30.0f, DT);
    close_to(c.d, -HEADING_KD * 30.0f, 0.0001f);
    assert(u < HEADING_KP * 10.0f);
    HeadingController_Reset(&c);
    (void)HeadingController_Update(&c, 0.0f, 0.0f, 30.0f, DT);
    close_to(c.d, -HEADING_KD * 30.0f, 0.0001f);   /* same D with no error */

    /* The rate low-pass: a single spike after priming moves D only a little. */
    HeadingController_Reset(&c);
    (void)HeadingController_Update(&c, 0.0f, 0.0f, 0.0f, DT);
    (void)HeadingController_Update(&c, 0.0f, 0.0f, 100.0f, DT);
    assert(fabsf(c.d) < HEADING_KD * 100.0f * 0.5f);

    /* Deadband: P rests, the integral is held and keeps its trim. */
    HeadingController_Reset(&c);
    (void)HeadingController_Update(&c, 10.0f, 0.0f, 0.0f, 1.0f);   /* integral 10 */
    close_to(c.integral, 10.0f, 0.001f);
    u = HeadingController_Update(&c, 1.0f, 0.0f, 0.0f, DT);
    close_to(u, HEADING_KI * 10.0f, 0.0005f);
    close_to(c.p, 0.0f, 0.0001f);
    close_to(c.integral, 10.0f, 0.001f);

    /* Saturation at +/-1 with anti-windup. */
    HeadingController_Reset(&c);
    close_to(HeadingController_Update(&c, 179.0f, 0.0f, 0.0f, DT), 1.0f, 0.0001f);
    close_to(c.integral, 0.0f, 0.001f);

    HeadingController_Reset(&c);
    close_to(c.integral, 0.0f, 0.001f);
    close_to(c.error_deg, 0.0f, 0.001f);

    /* Mix: positive u turns the bow right: rudders right, left motor up. */
    float left, right, steer;
    Heading_Mix(0.5f, 0.4f, &left, &right, &steer);
    close_to(steer, 0.5f * HEADING_RUDDER_GAIN, 0.0001f);
    close_to(left - right, 2.0f * 0.5f * HEADING_DIFF_GAIN * (float)HEADING_DIFF_DIR, 0.0001f);
    close_to((left + right) / 2.0f, 0.4f, 0.0001f);
    assert(HEADING_DIFF_DIR > 0 ? left > right : left < right);
    /* Low throttle: the split is capped so no motor goes below zero. */
    Heading_Mix(1.0f, 0.05f, &left, &right, &steer);
    assert(left >= 0.0f && right >= 0.0f && left <= 1.0f && right <= 1.0f);
    close_to(left + right, 0.10f, 0.0001f);
    /* No throttle: rudders only, motors stay off. */
    Heading_Mix(-0.7f, 0.0f, &left, &right, &steer);
    close_to(left, 0.0f, 0.0001f);
    close_to(right, 0.0f, 0.0001f);
    close_to(steer, -0.7f * HEADING_RUDDER_GAIN, 0.0001f);

    puts("heading_controller_test: PASS");
    return 0;
}
