/* Host test of App/front_motor.c: pushing with the rear motors, braking on a throttle cut. */
#include "front_motor.h"
#include "app_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static int near(float a, float b) { return fabsf(a - b) < 1e-5f; }

int main(void)
{
    front_motor_t f;
    FrontMotor_Init(&f);

    /* Standing still: nothing. Driving: it pushes with the rear motors. */
    assert(near(FrontMotor_Command(&f, 0.0f, 0U), 0.0f));
    assert(near(FrontMotor_Command(&f, 0.3f, 10U), 0.3f * FRONT_THRUST_GAIN));
    assert(near(FrontMotor_Command(&f, 0.3f, 20U), 0.3f * FRONT_THRUST_GAIN));

    /* Stop from 0.3: brakes for the reverse delay + 0.3 * MS_PER_THROTTLE, then rests. */
    uint32_t t = 1000U;
    uint32_t span = ESC_FRONT_REVERSE_DELAY_MS + (uint32_t)(0.3f * (float)FRONT_BRAKE_MS_PER_THROTTLE);
    assert(near(FrontMotor_Command(&f, 0.0f, t), -FRONT_BRAKE_LEVEL));
    assert(near(FrontMotor_Command(&f, 0.0f, t + span - 10U), -FRONT_BRAKE_LEVEL));
    assert(near(FrontMotor_Command(&f, 0.0f, t + span), 0.0f));
    assert(near(FrontMotor_Command(&f, 0.0f, t + span + 500U), 0.0f));

    /* A small easing off (below MIN_DROP) does not brake. */
    FrontMotor_Command(&f, 0.30f, 5000U);
    assert(near(FrontMotor_Command(&f, 0.28f, 5010U), 0.28f * FRONT_THRUST_GAIN));

    /* Slowing 0.5 -> 0.2 brakes while the rear motors keep 0.2; speeding up ends it at once. */
    FrontMotor_Command(&f, 0.5f, 6000U);
    assert(near(FrontMotor_Command(&f, 0.2f, 6010U), -FRONT_BRAKE_LEVEL));
    assert(near(FrontMotor_Command(&f, 0.2f, 6100U), -FRONT_BRAKE_LEVEL));
    assert(near(FrontMotor_Command(&f, 0.4f, 6110U), 0.4f * FRONT_THRUST_GAIN));

    /* The braking time is capped. */
    FrontMotor_Command(&f, 1.0f, 7000U);
    FrontMotor_Command(&f, 0.0f, 7010U);
    assert(near(FrontMotor_Command(&f, 0.0f, 7010U + ESC_FRONT_REVERSE_DELAY_MS + FRONT_BRAKE_MAX_MS), 0.0f));

    /* A fault stop resets it: no braking afterwards. */
    FrontMotor_Command(&f, 0.4f, 9000U);
    FrontMotor_Reset(&f);
    assert(near(FrontMotor_Command(&f, 0.0f, 9010U), 0.0f));

    /* The tick counter wrapping round does not stretch the brake. */
    FrontMotor_Command(&f, 0.3f, 0xFFFFFF00U);
    assert(near(FrontMotor_Command(&f, 0.0f, 0xFFFFFF10U), -FRONT_BRAKE_LEVEL));
    assert(near(FrontMotor_Command(&f, 0.0f, 0xFFFFFF10U + span), 0.0f));

    printf("front_motor_test: PASS\n");
    return 0;
}
