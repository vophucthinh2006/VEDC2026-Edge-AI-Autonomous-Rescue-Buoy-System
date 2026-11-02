#pragma once

/* Time slots on the one LoRa channel several boats share (docs/fleet-control.md).
 *
 * Every boat beacons every LORA_SLOT_FRAME_MS, inside a slot of its own: PHAO-01 in slot 0, PHAO-02 in
 * slot 1, and so on. A slot holds the beacon (about 0.7 s on air at SF9/BW125), the station's answer
 * (an RTE frame takes about 0.8 s) and the turnaround, so two boats never talk over each other and the
 * station answers each one in turn. The clock is the GPS time of day every boat has, no message from
 * the station is needed to share it.
 *
 * Plain C, no HAL: the host tests and the simulator (sim/tools/route_radio_shim.c) use it as it is.
 * The real boat sails alone and keeps its free-running beacon (LORA_BEACON_PERIOD_MS); the simulator
 * runs the fleet on these slots. */

#include <stdint.h>

#define LORA_SLOT_MS        2000U   /* beacon + answer + margin */
#define LORA_SLOT_COUNT     4U      /* boats on one channel */
#define LORA_SLOT_FRAME_MS  (LORA_SLOT_MS * LORA_SLOT_COUNT)

/* The slot of a boat from its name: the number at its end, minus one ("PHAO-03" -> 2), modulo
 * slots; a name without one gets a slot from a hash of it. */
uint8_t LoraSlot_FromId(const char *id, uint8_t slots);

/* Milliseconds from clock_ms (the shared clock, ms of the GPS day) to the start of the boat's next slot;
 * 0 when clock_ms is that start. */
uint32_t LoraSlot_Wait(uint32_t clock_ms, uint8_t slot, uint8_t slots, uint32_t slot_ms);
