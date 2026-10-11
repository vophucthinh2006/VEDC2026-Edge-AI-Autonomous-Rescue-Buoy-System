#include "helper_logic.h"
#include "app_config.h"
#include "lora_frame.h"
#include <stdio.h>
#include <string.h>

/* Signed: an interrupt can stamp a tick newer than the now_ms the loop read. */
static bool within(uint32_t now_ms, uint32_t stamp_ms, uint32_t timeout_ms) {
    return (int32_t)(now_ms - stamp_ms) <= (int32_t)timeout_ms;
}

void HelperLogic_Init(helper_logic_t *logic) {
    memset(logic, 0, sizeof(*logic));
    logic->out.servo_us = CAM_CENTER_US;
}

void HelperLogic_Fast(helper_logic_t *logic, const pi_link_t *pi, const helper_inputs_t *in, uint32_t now_ms) {
    helper_outputs_t *out = &logic->out;
    out->pi_ok = pi->seen && within(now_ms, pi->last_hbt_ms, PI_LINK_TIMEOUT_MS);

    /* Camera servo: the Pi's angle while it keeps coming, straight ahead otherwise. */
    int32_t pan = pi->cam_seen && within(now_ms, pi->last_cam_ms, CAM_TIMEOUT_MS) ? pi->cam_pan_ddeg : 0;
    if (pan > CAM_PAN_LIMIT_DDEG) pan = CAM_PAN_LIMIT_DDEG;
    if (pan < -CAM_PAN_LIMIT_DDEG) pan = -CAM_PAN_LIMIT_DDEG;
    out->servo_us = (uint16_t)(CAM_CENTER_US + CAM_PAN_DIR * pan * CAM_US_PER_10DEG / 100);

    /* Alarm: a higher count than the last one seen. A lower one is the Pi starting again. */
    uint8_t victims = pi->victims;
    if (victims > logic->seen_victims) {
        logic->alarm_until_ms = now_ms + ALARM_HOLD_MS;
        logic->alarm_on = true;
        logic->beacon_now = true;         /* the shore hears of it at once, not at the next period */
    }
    logic->seen_victims = victims;
    if (logic->alarm_on && (int32_t)(now_ms - logic->alarm_until_ms) >= 0) logic->alarm_on = false;
    out->relays_on = logic->alarm_on;
    out->victims = victims;

    /* E-stop button, debounced over ESTOP_DEBOUNCE_TICKS samples. */
    if (!in->estop_pressed) logic->estop_ticks = 0U;
    else if (logic->estop_ticks < ESTOP_DEBOUNCE_TICKS) logic->estop_ticks++;
    out->estop = logic->estop_ticks >= ESTOP_DEBOUNCE_TICKS;

    /* LED: one blink a second with the Pi talking, five without. */
    uint32_t half = out->pi_ok ? 500U : 100U;
    out->led_on = (now_ms / half) % 2U == 0U;
}

bool HelperLogic_TakeBeaconRequest(helper_logic_t *logic) {
    bool request = logic->beacon_now;
    logic->beacon_now = false;
    return request;
}

size_t HelperLogic_Beacon(const helper_logic_t *logic, const char *status, bool status_seen, uint32_t last_status_ms,
                          uint32_t now_ms, uint32_t sequence, char *frame, size_t size) {
    bool fresh = status_seen && within(now_ms, last_status_ms, PI_STATUS_STALE_MS);
    return LoraFrame_Build(frame, size, LORA_BUOY_ID, fresh ? status : NULL, sequence, logic->seen_victims);
}

size_t HelperLogic_Report(helper_logic_t *logic, const helper_inputs_t *in, char *line, size_t size) {
    char payload[48];
    snprintf(payload, sizeof(payload), "%u,%u,%ld,%u,%u,%u", (unsigned int)++logic->report_sequence,
             (unsigned int)(in->ina_ok ? in->bus_mv : 0U), (long)(in->ina_ok ? in->current_ma : 0),
             logic->out.estop ? 1U : 0U, in->lora_ok ? 1U : 0U, logic->out.relays_on ? 1U : 0U);
    return PiLink_Encode(line, size, "AUX", payload);
}
