#pragma once

/* Route sent to a buoy (docs/lora-route-downlink.md): the copy the admin sent from the dashboard,
 * fetched from GET /api/downlink, and the RTE frame that answers a beacon. Plain C, no ESP-IDF:
 * the host test and the simulator use it as it is. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DOWNLINK_MAX_POINTS  300U
#define DOWNLINK_FRAME_MAX   120U   /* bytes on air; the buoy's buffer takes 127 */
#define DOWNLINK_SLOTS       4U     /* buoys the station holds a route for at the same time */

/* The admin's commands in an ACK (cmd field): bits 0-1 the two lights, then: */
#define DOWNLINK_CMD_STOP    4U     /* the F407 keeps the motors off, the route stays */
#define DOWNLINK_CMD_RETURN  8U     /* the Pi leaves its route and goes to its parking spot (lat, lon) */

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

/* A buoy's place in a formation (docs/fleet-control.md): it keeps right_m to the right of and back_m
 * behind the leader, in the leader's own frame (negative: left, ahead). on = false: no formation. */
typedef struct {
    bool on;
    char leader[32];
    int16_t right_m;
    int16_t back_m;
} formation_role_t;

/* The answer to every other frame of a buoy (STM32F4/App/shore_link.h):
 *     ACK,<id>,<eseq>,<cmd>,<lat>,<lon>[,<leader>,<right_m>,<back_m>]*CS
 * eseq: the event frame acknowledged, 0 for a beacon. cmd: the admin's commands 0..15, bit 0 light 1
 * (PC3), bit 1 light 2 (PC2), DOWNLINK_CMD_STOP, DOWNLINK_CMD_RETURN. lat, lon: the buoy's way home,
 * its own parking spot or the station's GPS; empty without one. The last three fields only while the
 * buoy follows a leader (role on), so a buoy outside any formation gets the frame older firmware takes.
 * The buoy takes any frame from the station as a sign the station is there. */
size_t RouteDownlink_Ack(const char *id, uint16_t eseq, uint8_t cmd, bool has_position, double lat, double lon,
                         const formation_role_t *role, char *buf, size_t size);

/* The route among routes[count] that is for buoy id, or NULL. */
const route_downlink_t *RouteDownlink_Find(const route_downlink_t *routes, size_t count, const char *id);

/* Parses the "rev:have:pi" value of a beacon's w= field. */
bool RouteDownlink_ParseAck(const char *value, route_ack_t *ack);
