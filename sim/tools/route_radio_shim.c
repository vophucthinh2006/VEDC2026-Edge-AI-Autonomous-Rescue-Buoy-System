/* Built with STM32F4/App/route_link.c and esp32-lora-station/main/route_downlink.c into
 * librouteradio.so (tools/hw_standins.sh), so the simulated STM32F407 and shore station run the
 * firmware's own route code through ctypes. Only what Python cannot reach by itself is here:
 * the struct sizes and filling the station's copy of the route. */
#include <string.h>
#include "route_link.h"
#include "route_downlink.h"
#include "victim_claims.h"

size_t sim_route_link_size(void) { return sizeof(route_link_t); }
size_t sim_route_downlink_size(void) { return sizeof(route_downlink_t); }
size_t sim_claims_size(void) { return sizeof(claims_t); }
size_t sim_claim_size(void) { return sizeof(claim_t); }

uint16_t sim_route_link_rev(const route_link_t *route) { return route->rev; }
uint16_t sim_route_link_have(const route_link_t *route) { return route->have; }
uint16_t sim_route_link_total(const route_link_t *route) { return route->total; }

/* What the station keeps from GET /api/downlink. False if it does not fit. */
bool sim_route_downlink_set(route_downlink_t *route, uint16_t rev, const char *id, bool loop,
                            uint16_t count, const double *lat, const double *lon)
{
    memset(route, 0, sizeof(*route));
    if (count > DOWNLINK_MAX_POINTS || strlen(id) >= sizeof(route->id)) return false;
    route->rev = rev;
    strcpy(route->id, id);
    route->loop = loop;
    route->count = count;
    memcpy(route->lat, lat, count * sizeof(double));
    memcpy(route->lon, lon, count * sizeof(double));
    return true;
}

/* The shore link and the event queue of the F407 (App/shore_link.c, App/event_queue.c), and the
 * station's event parser (lora_payload.c), for the same reason. App/lora_slot.c (the fleet's time
 * slots) needs no help: Python calls LoraSlot_FromId and LoraSlot_Wait as they are. */
#include <stdio.h>
#include "lora_slot.h"
#include "shore_link.h"
#include "event_queue.h"
#include "lora_payload.h"

size_t sim_shore_link_size(void) { return sizeof(shore_link_t); }
size_t sim_event_queue_size(void) { return sizeof(event_queue_t); }

uint8_t sim_shore_lights(const shore_link_t *shore) { return shore->lights; }
bool sim_shore_hold(const shore_link_t *shore) { return shore->hold; }

bool sim_shore_home(const shore_link_t *shore, double *lat, double *lon)
{
    *lat = shore->home_lat_e6 / 1e6;
    *lon = shore->home_lon_e6 / 1e6;
    return shore->home_valid;
}

uint8_t sim_event_count(const event_queue_t *queue) { return queue->count; }

/* RETURN and the formation of the dashboard (docs/fleet-control.md), as App/shore_link.c took them. */
bool sim_shore_ret(const shore_link_t *shore) { return shore->ret; }

bool sim_shore_follow(const shore_link_t *shore, char *leader, size_t size, int16_t *right_m, int16_t *back_m)
{
    if (!shore->follow || size == 0U) return false;
    snprintf(leader, size, "%s", shore->leader);
    *right_m = shore->right_m;
    *back_m = shore->back_m;
    return true;
}

/* A beacon of another boat (the leader of a formation), with the station's own parser (lora_payload.c):
 * position, compass heading, course and speed over ground. False if it is not a beacon with a position. */
bool sim_beacon_parse(const char *frame, char *id, size_t id_size, double *lat, double *lon, float *yaw,
                      float *course, float *speed)
{
    lora_payload_t p;
    if (!lora_payload_parse(frame, &p) || !p.has_position || id_size == 0U) return false;
    snprintf(id, id_size, "%s", p.id);
    *lat = p.lat;
    *lon = p.lon;
    *yaw = p.has_attitude ? p.yaw : -1.0f;
    *course = p.has_course ? p.course : -1.0f;
    *speed = p.has_speed ? p.speed : -1.0f;
    return true;
}

/* The "event" group the station posts for an ev= frame (esp32-lora-station/main/main.c
 * xu_ly_su_kien), with the buoy id in front: "PHAO-01\t{...}". 0 if the frame is not an event. */
size_t sim_event_json(const char *frame, char *buf, size_t size)
{
    lora_event_t ev;
    if (!lora_event_parse(frame, &ev)) return 0U;
    char lat[24] = "null", lon[24] = "null";
    if (ev.has_position) {
        snprintf(lat, sizeof(lat), "%.6f", ev.lat);
        snprintf(lon, sizeof(lon), "%.6f", ev.lon);
    }
    int n = snprintf(buf, size, "%s\t{\"seq\":%u,\"type\":\"%c\",\"lat\":%s,\"lon\":%s", ev.id, ev.seq, ev.type, lat, lon);
    if (n < 0 || (size_t)n >= size) return 0U;
    int m;
    if (ev.type == 'V')
        m = snprintf(buf + n, size - (size_t)n, ",\"risk\":%u,\"conf\":%.2f,\"count\":%u}", ev.risk, ev.confidence, ev.count);
    else if (ev.type == 'T')
        m = snprintf(buf + n, size - (size_t)n, ",\"roll\":%.1f,\"pitch\":%.1f}", ev.roll, ev.pitch);
    else
        m = snprintf(buf + n, size - (size_t)n, ",\"presses\":%u}", ev.presses);
    if (m < 0 || (size_t)(n + m) >= size) return 0U;
    return (size_t)(n + m);
}
