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

/* The answer to every other frame of a buoy (STM32F4/App/shore_link.h):
 *     ACK,<id>,<eseq>,<cmd>,<lat>,<lon>*CS
 * eseq: the event frame acknowledged, 0 for a beacon. cmd: the admin's commands 0..7, bit 0 light 1
 * (PC3), bit 1 light 2 (PC2), bit 2 STOP (the F407 keeps the motors off). lat, lon: the station's GPS, the buoy's way home; empty without a fix.
 * The buoy takes any frame from the station as a sign the station is there. */
size_t RouteDownlink_Ack(const char *id, uint16_t eseq, uint8_t lights, bool has_position, double lat, double lon,
                         char *buf, size_t size);

/* Parses the "rev:have:pi" value of a beacon's w= field. */
bool RouteDownlink_ParseAck(const char *value, route_ack_t *ack);
