#pragma once

/* What the boat knows of the shore station (docs/lora-route-downlink.md, "ACK").
 * The station answers every frame of the boat: with an RTE frame while a route is still going
 * across, otherwise with
 *
 *     ACK,<id>,<eseq>,<cmd>,<lat>,<lon>*CS
 *
 * eseq: the event frame being acknowledged (App/event_queue.h), 0 for a plain beacon.
 * cmd: the admin's commands from the dashboard, 0..7. Bit 0 = RELAY1 PC3, bit 1 = RELAY2 PC2 (the
 * lights); bit 2 = STOP: the motors stay off until the admin presses "Chạy tiếp", whatever the Pi asks.
 * lat, lon: the station's own GPS, the boat's way home; both empty while the station has no fix.
 *
 * Any frame from the station counts as hearing it. Silent for SHORE_LINK_TIMEOUT_MS after it was
 * heard once, the link is lost and the Pi takes the boat home ($LNK). Never heard yet counts as fine:
 * without the station there is no route either, and the boat stands still anyway.
 * Plain C, no HAL: the host tests and the simulator use it as it is. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SHORE_LINK_TIMEOUT_MS  60000U

typedef struct {
    bool ever_heard;
    uint32_t last_heard_ms;
    bool home_valid;           /* the station's position came in at least once */
    int32_t home_lat_e6;
    int32_t home_lon_e6;
    bool lights_known;         /* an ACK came in since boot: lights and hold hold the admin's commands */
    uint8_t lights;
    bool hold;                 /* STOP from the dashboard: motors off (App/control.c) */
    uint32_t acks_ok;          /* ACK frames taken (debug) */
    uint32_t acks_bad;         /* ACK frames for this buoy that were refused */
} shore_link_t;

void ShoreLink_Init(shore_link_t *shore);

/* Any frame of the station for this buoy (an RTE frame RouteLink_OnFrame took, or an ACK). */
void ShoreLink_Heard(shore_link_t *shore, uint32_t now_ms);

/* One LoRa frame. True if it was an ACK for buoy_id that was taken; *eseq is then the event it
 * acknowledges (0: none). Takes the lights and, when present, the station's position. */
bool ShoreLink_OnFrame(shore_link_t *shore, const char *frame, const char *buoy_id, uint32_t now_ms, uint16_t *eseq);

/* False once the station, heard before, has been silent for SHORE_LINK_TIMEOUT_MS. */
bool ShoreLink_Ok(const shore_link_t *shore, uint32_t now_ms);

/* Payload of the $LNK line for the Pi: "seq,shore_ok,heard_ago_s,home_valid,home_lat,home_lon,hold"
 * (heard_ago_s -1 while never heard; lat and lon empty without a home; hold 1 while stopped from the
 * dashboard). Returns the length, 0 if it did not fit. */
size_t ShoreLink_PiLine(const shore_link_t *shore, uint16_t seq, uint32_t now_ms, char *buf, size_t size);
