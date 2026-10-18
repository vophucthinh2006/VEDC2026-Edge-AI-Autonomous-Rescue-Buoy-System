#pragma once

/* Route sent from the dashboard over LoRa (docs/lora-route-downlink.md).
 * Assembles the shore station's RTE frames into a route, reports progress in the beacon
 * ("w=rev:have:pi") and hands the finished route to the Pi as $RTE lines until the Pi's $HBT
 * reports the same revision. Plain C, no HAL: the host tests and the simulator use it as it is. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ROUTE_MAX_POINTS        300U
#define ROUTE_PI_POINTS_PER_LINE 7U      /* "$RTE,..." stays under the 192-byte UART line */
#define ROUTE_PI_RESEND_MS      2000U    /* pause between two passes while the Pi has not confirmed */

typedef struct {
    int32_t lat_e6;
    int32_t lon_e6;
} route_point_t;

typedef struct {
    uint16_t rev;              /* route being received or held, 0 = none */
    uint16_t total;
    uint16_t have;             /* points received in order from the first */
    bool loop;
    uint16_t pi_rev;           /* revision the Pi reports in $HBT */
    uint16_t pi_next;          /* first point of the next $RTE line */
    uint32_t pi_pause_until_ms;
    bool pi_paused;
    uint32_t frames_ok;        /* RTE frames taken (debug) */
    uint32_t frames_bad;       /* RTE frames for this buoy that were refused */
    route_point_t points[ROUTE_MAX_POINTS];
} route_link_t;

void RouteLink_Init(route_link_t *route);

/* One LoRa frame. True if it was an RTE frame for buoy_id that was taken. */
bool RouteLink_OnFrame(route_link_t *route, const char *frame, const char *buoy_id);

static inline bool RouteLink_Complete(const route_link_t *route) { return route->rev != 0U && route->have >= route->total; }
static inline bool RouteLink_Receiving(const route_link_t *route) { return route->rev != 0U && route->have < route->total; }

/* "w=rev:have:pi" for the beacon, or an empty string while no route was ever received.
 * Returns the length written. */
size_t RouteLink_BeaconField(const route_link_t *route, char *buf, size_t size);

/* The revision in the Pi's $HBT (0 when the Pi sends none). */
void RouteLink_SetPiRev(route_link_t *route, uint16_t rev);

/* The payload of the next $RTE line ("seq,rev,first,total,loop,lat;lon,..."), or false when there
 * is nothing to send now. Call it once per telemetry period. */
bool RouteLink_NextPiLine(route_link_t *route, uint16_t seq, uint32_t now_ms, char *buf, size_t size);
