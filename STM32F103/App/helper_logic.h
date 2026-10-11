#pragma once

/* What the helper board decides, with no HAL in it: the firmware (App/app.c) feeds it pins and
 * sensor readings and carries out what it answers; the simulator (Tests/helper_sim.c) runs the
 * very same code against the Pi program on a PC. */

#include "pi_link.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    bool estop_pressed;        /* raw level of the button, before debouncing */
    bool ina_ok;
    uint16_t bus_mv;
    int32_t current_ma;
    bool lora_ok;              /* radio initialised */
} helper_inputs_t;

typedef struct {
    uint16_t servo_us;
    bool relays_on;            /* alarm */
    bool led_on;
    bool pi_ok;                /* HBT within PI_LINK_TIMEOUT_MS */
    bool estop;                /* debounced */
    uint8_t victims;
} helper_outputs_t;

typedef struct {
    uint8_t seen_victims;
    uint8_t estop_ticks;
    bool alarm_on;
    bool beacon_now;           /* the count rose: send a beacon without waiting for the period */
    uint32_t alarm_until_ms;
    uint16_t report_sequence;
    helper_outputs_t out;      /* as of the last HelperLogic_Fast */
} helper_logic_t;

void HelperLogic_Init(helper_logic_t *logic);
/* Every HELPER_FAST_PERIOD_MS. pi may be written by an interrupt meanwhile: only single fields
   are read here. */
#define HELPER_FAST_PERIOD_MS 10U
void HelperLogic_Fast(helper_logic_t *logic, const pi_link_t *pi, const helper_inputs_t *in, uint32_t now_ms);
/* True once after the victim count rose. */
bool HelperLogic_TakeBeaconRequest(helper_logic_t *logic);
/* The LoRa line. status / status_seen / last_status_ms: a consistent copy of the Pi link's
   fields (the firmware copies them with the UART interrupt held off). Returns its length. */
size_t HelperLogic_Beacon(const helper_logic_t *logic, const char *status, bool status_seen, uint32_t last_status_ms,
                          uint32_t now_ms, uint32_t sequence, char *frame, size_t size);
/* The "$AUX,..." line for the Pi. Returns its length. */
size_t HelperLogic_Report(helper_logic_t *logic, const helper_inputs_t *in, char *line, size_t size);
