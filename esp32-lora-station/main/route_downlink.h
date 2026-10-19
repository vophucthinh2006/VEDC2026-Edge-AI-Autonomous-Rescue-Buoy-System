#pragma once

/* Route sent to a buoy (docs/lora-route-downlink.md): the copy the admin sent from the dashboard,
 * fetched from GET /api/downlink, and the RTE frame that answers a beacon. Plain C, no ESP-IDF:
 * the host test and the simulator use it as it is. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DOWNLINK_MAX_POINTS  300U
#define DOWNLINK_FRAME_MAX   120U   /* bytes on air; the buoy's buffer takes 127 */

typedef struct {
    uint16_t rev;                   /* 0: nothing to send */
    char id[32];                    /* buoy it is for */
    bool loop;
    uint16_t count;
    double lat[DOWNLINK_MAX_POINTS];
    double lon[DOWNLINK_MAX_POINTS];
} route_downlink_t;

/* What the buoy's beacon reported ("w=rev:have:pi"); has = false for a beacon without it. */
typedef struct {
    bool has;
    uint16_t rev;
    uint16_t have;
    uint8_t pi;
} route_ack_t;

/* The RTE frame to send right after a beacon from beacon_id that reported ack, or 0 when the
 * buoy already has the route (or it is not for that buoy). buf needs DOWNLINK_FRAME_MAX + 1 bytes. */
size_t RouteDownlink_Frame(const route_downlink_t *route, const char *beacon_id, const route_ack_t *ack,
                           char *buf, size_t size);

/* Parses the "rev:have:pi" value of a beacon's w= field. */
bool RouteDownlink_ParseAck(const char *value, route_ack_t *ack);
