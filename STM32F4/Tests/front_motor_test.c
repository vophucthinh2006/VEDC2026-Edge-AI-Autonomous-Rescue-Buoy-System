/* Host test of App/front_motor.c: pushing with the rear motors, braking on a throttle cut. */
#include "front_motor.h"
#include "app_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static int near(float a, float b) { return fabsf(a - b) < 1e-5f; }

/* The control loop: one call every 10 ms for ms; returns the last output. */
static float drive(front_motor_t *f, float throttle, uint32_t *now, uint32_t ms)
{
    float out = 0.0f;
    for (uint32_t t = 0U; t < ms; t += 10U) { out = FrontMotor_Command(f, throttle, *now); *now += 10U; }
    return out;
}

/* Times the output turns to the brake while throttle alternates with 0 every 100 ms. */
static unsigned brakes_while_flickering(front_motor_t *f, float high, float low, uint32_t *now, uint32_t ms)
{
    unsigned brakes = 0U;
    bool was = false;
    for (uint32_t t = 0U; t < ms; t += 10U) {
        float out = FrontMotor_Command(f, (t / 100U) % 2U ? low : high, *now);
        bool braking = near(out, -FRONT_BRAKE_LEVEL);
        if (braking && !was) brakes++;
        was = braking;
        *now += 10U;
    }
    return brakes;
}

int main(void)
{
    front_motor_t f;
    FrontMotor_Init(&f);
    uint32_t now = 0U;

    /* Standing still: nothing. Driving: it pushes with the rear motors. */
    assert(near(drive(&f, 0.0f, &now, 500U), 0.0f));
    assert(near(drive(&f, 0.3f, &now, 3000U), 0.3f * FRONT_THRUST_GAIN));

    /* Stop after 0.3 held: brakes for the reverse delay + 0.3 * MS_PER_THROTTLE, then rests. */
    uint32_t t = now;
    uint32_t span = ESC_FRONT_REVERSE_DELAY_MS + (uint32_t)(0.3f * (float)FRONT_BRAKE_MS_PER_THROTTLE);
    assert(near(FrontMotor_Command(&f, 0.0f, t), -FRONT_BRAKE_LEVEL));
    assert(near(FrontMotor_Command(&f, 0.0f, t + span - 10U), -FRONT_BRAKE_LEVEL));
    assert(near(FrontMotor_Command(&f, 0.0f, t + span), 0.0f));
    now = t + span + 10U;
    assert(near(drive(&f, 0.0f, &now, 500U), 0.0f));

    /* A small easing off (below MIN_DROP) does not brake, nor a slow one step by step. */
    drive(&f, 0.30f, &now, 3000U);
    assert(near(FrontMotor_Command(&f, 0.28f, now), 0.28f * FRONT_THRUST_GAIN));
    for (int i = 0; i < 10; i++) {
        now += 100U;
        assert(!near(FrontMotor_Command(&f, 0.28f - 0.02f * (float)(i + 1), now), -FRONT_BRAKE_LEVEL));
    }

    /* Slowing 0.5 -> 0.2 brakes while the rear motors keep 0.2; speeding up ends it at once. */
    drive(&f, 0.5f, &now, 3000U);
    assert(near(FrontMotor_Command(&f, 0.2f, now), -FRONT_BRAKE_LEVEL));
    assert(near(FrontMotor_Command(&f, 0.2f, now + 90U), -FRONT_BRAKE_LEVEL));
    assert(near(FrontMotor_Command(&f, 0.4f, now + 100U), 0.4f * FRONT_THRUST_GAIN));
    now += 110U;

    /* The braking time is capped. */
    drive(&f, 1.0f, &now, 3000U);
    t = now;
    assert(near(FrontMotor_Command(&f, 0.0f, t), -FRONT_BRAKE_LEVEL));
    assert(near(FrontMotor_Command(&f, 0.0f, t + ESC_FRONT_REVERSE_DELAY_MS + FRONT_BRAKE_MAX_MS), 0.0f));
    now = t + 2000U;

    /* A fault stop resets it: no braking afterwards. */
    drive(&f, 0.4f, &now, 3000U);
    FrontMotor_Reset(&f);
    assert(near(FrontMotor_Command(&f, 0.0f, now), 0.0f));
    drive(&f, 0.0f, &now, 1000U);

    /* A blip of throttle between two stops: the boat has no speed to brake off. */
    drive(&f, 0.0f, &now, 2000U);
    drive(&f, 0.3f, &now, 300U);
    assert(near(FrontMotor_Command(&f, 0.0f, now), 0.0f));
    drive(&f, 0.0f, &now, 1000U);

    /* The Pi flickering between STOP and a creep (obstacle_too_close), or between approach and the
       stand-off: one brake for the speed the boat had, not a reversal every 100 ms. */
    drive(&f, 0.15f, &now, 3000U);
    assert(brakes_while_flickering(&f, 0.15f, 0.0f, &now, 5000U) == 1U);
    drive(&f, 0.0f, &now, 2000U);
    assert(brakes_while_flickering(&f, 0.15f, 0.0f, &now, 5000U) == 0U);
    drive(&f, 0.0f, &now, 2000U);
    /* Between approach (0.15) and facing the person (0.10): no brake at all after the first. */
    drive(&f, 0.15f, &now, 3000U);
    assert(brakes_while_flickering(&f, 0.15f, 0.10f, &now, 5000U) <= 1U);
    assert(brakes_while_flickering(&f, 0.15f, 0.10f, &now, 5000U) == 0U);

    /* The tick counter wrapping round neither stops the floor rising nor stretches the brake. */
    drive(&f, 0.0f, &now, 2000U);
    now = 0xFFFFFFFFU - 1500U;
    drive(&f, 0.3f, &now, 3000U);   /* the hold windows run across the wrap */
    assert(near(drive(&f, 0.0f, &now, 10U), -FRONT_BRAKE_LEVEL));
    drive(&f, 0.0f, &now, 2000U);
    now = 0xFFFFFFFFU - 3400U;
    drive(&f, 0.3f, &now, 3000U);
    t = now;                        /* the brake runs across the wrap */
    assert(near(FrontMotor_Command(&f, 0.0f, t), -FRONT_BRAKE_LEVEL));
    assert(near(FrontMotor_Command(&f, 0.0f, t + span - 10U), -FRONT_BRAKE_LEVEL));
    assert(near(FrontMotor_Command(&f, 0.0f, t + span), 0.0f));

    printf("front_motor_test: PASS\n");
    return 0;
}
