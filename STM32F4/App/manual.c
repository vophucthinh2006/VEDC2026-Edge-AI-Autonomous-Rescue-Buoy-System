#include "manual.h"
#include "app_config.h"

static float clamp(float value, float low, float high) { return value < low ? low : (value > high ? high : value); }

/* Self-centring stick to -1..1, zero inside the deadband and rising from
   there, so the first bit of travel past it is not a jump. */
static float stick(uint16_t us) {
    float offset = (float)us - 1500.0f;
    float span = 500.0f - (float)RC_STICK_DEADBAND_US;
    if (offset > (float)RC_STICK_DEADBAND_US) return clamp((offset - (float)RC_STICK_DEADBAND_US) / span, 0.0f, 1.0f);
    if (offset < -(float)RC_STICK_DEADBAND_US) return clamp((offset + (float)RC_STICK_DEADBAND_US) / span, -1.0f, 0.0f);
    return 0.0f;
}

bool Manual_SpeedCentred(const ibus_state_t *rc) { return stick(rc->channel[RC_CH_SPEED]) == 0.0f; }

void Manual_Mix(const ibus_state_t *rc, actuator_cmd_t *cmd) {
    float power = clamp(((float)rc->channel[RC_CH_POWER] - (float)RC_POWER_MIN_US) / (2000.0f - (float)RC_POWER_MIN_US), 0.0f, 1.0f);
    float speed = stick(rc->channel[RC_CH_SPEED]) * power;
    if (speed >= 0.0f) {
        cmd->rear_left = cmd->rear_right = speed * MANUAL_REAR_MAX;
        cmd->front = speed * MANUAL_FRONT_FWD_GAIN;
    } else {
        /* The rear ESCs cannot reverse; the front motor brakes and backs up. */
        cmd->rear_left = cmd->rear_right = 0.0f;
        cmd->front = speed * MANUAL_FRONT_REV_MAX;
    }
    cmd->steer = stick(rc->channel[RC_CH_STEER]);
}
