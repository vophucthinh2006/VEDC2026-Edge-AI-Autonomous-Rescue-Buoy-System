/* Built with STM32F4/App/route_link.c and esp32-lora-station/main/route_downlink.c into
 * librouteradio.so (tools/hw_standins.sh), so the simulated STM32F407 and shore station run the
 * firmware's own route code through ctypes. Only what Python cannot reach by itself is here:
 * the struct sizes and filling the station's copy of the route. */
#include <string.h>
#include "route_link.h"
#include "route_downlink.h"

size_t sim_route_link_size(void) { return sizeof(route_link_t); }
size_t sim_route_downlink_size(void) { return sizeof(route_downlink_t); }

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
