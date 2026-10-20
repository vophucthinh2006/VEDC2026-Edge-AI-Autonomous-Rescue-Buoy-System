#include "route_downlink.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

size_t RouteDownlink_Frame(const route_downlink_t *route, const char *beacon_id, const route_ack_t *ack,
                           char *buf, size_t size)
{
    if (size == 0U) return 0U;
    buf[0] = '\0';
    if (route->rev == 0U || route->count > DOWNLINK_MAX_POINTS || strcmp(route->id, beacon_id) != 0) return 0U;
    uint16_t first = 0U;
    if (ack != NULL && ack->has && ack->rev == route->rev) {
        if (ack->have >= route->count) return 0U;   /* the buoy has all of it */
        first = ack->have;
    }
    /* Room for the frame without its "*CS". */
    size_t limit = (size - 1U < DOWNLINK_FRAME_MAX ? size - 1U : DOWNLINK_FRAME_MAX) - 3U;
    int n = snprintf(buf, size, "RTE,%s,%u,%u,%u,%u", route->id, (unsigned)route->rev, (unsigned)first,
                     (unsigned)route->count, route->loop ? 1U : 0U);
    if (n < 0 || (size_t)n > limit) { buf[0] = '\0'; return 0U; }
    size_t used = (size_t)n;
    for (uint16_t i = first; i < route->count; i++) {
        char point[32];
        int p = snprintf(point, sizeof(point), ",%.6f,%.6f", route->lat[i], route->lon[i]);
        if (p < 0 || used + (size_t)p > limit) break;
        memcpy(buf + used, point, (size_t)p + 1U);
        used += (size_t)p;
    }
    /* XOR of every character before '*', as on the Pi UART: one wrong digit is a wrong waypoint,
       and the station's radio sends without LoRa CRC unless told to. */
    uint8_t cs = 0U;
    for (size_t i = 0U; i < used; i++) cs ^= (uint8_t)buf[i];
    snprintf(buf + used, size - used, "*%02X", cs);
    return used + 3U;
}

size_t RouteDownlink_Ack(const char *id, uint16_t eseq, uint8_t lights, bool has_position, double lat, double lon,
                         char *buf, size_t size)
{
    if (size == 0U) return 0U;
    int n = has_position
          ? snprintf(buf, size, "ACK,%s,%u,%u,%.6f,%.6f", id, (unsigned)eseq, (unsigned)(lights & 7U), lat, lon)
          : snprintf(buf, size, "ACK,%s,%u,%u,,", id, (unsigned)eseq, (unsigned)(lights & 7U));
    if (n < 0 || (size_t)n + 3U >= size || (size_t)n + 3U > DOWNLINK_FRAME_MAX) { buf[0] = '\0'; return 0U; }
    uint8_t cs = 0U;
    for (int i = 0; i < n; i++) cs ^= (uint8_t)buf[i];
    snprintf(buf + n, size - (size_t)n, "*%02X", cs);
    return (size_t)n + 3U;
}

bool RouteDownlink_ParseAck(const char *value, route_ack_t *ack)
{
    memset(ack, 0, sizeof(*ack));
    char *end = NULL;
    unsigned long rev = strtoul(value, &end, 10);
    if (end == value || *end != ':' || rev == 0UL || rev > 0xFFFFUL) return false;
    const char *next = end + 1;
    unsigned long have = strtoul(next, &end, 10);
    if (end == next || *end != ':' || have > DOWNLINK_MAX_POINTS) return false;
    next = end + 1;
    unsigned long pi = strtoul(next, &end, 10);
    if (end == next || *end != '\0' || pi > 1UL) return false;
    ack->has = true;
    ack->rev = (uint16_t)rev;
    ack->have = (uint16_t)have;
    ack->pi = (uint8_t)pi;
    return true;
}
