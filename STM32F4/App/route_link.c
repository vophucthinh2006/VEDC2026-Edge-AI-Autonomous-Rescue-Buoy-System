#include "route_link.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_FRAME 128U

void RouteLink_Init(route_link_t *route)
{
    memset(route, 0, sizeof(*route));
}

static bool parse_uint(const char *text, unsigned long max, unsigned long *out)
{
    char *end = NULL;
    if (text == NULL || *text < '0' || *text > '9') return false;
    unsigned long value = strtoul(text, &end, 10);
    if (*end != '\0' || value > max) return false;
    *out = value;
    return true;
}

static bool parse_coord(const char *text, double limit, int32_t *out)
{
    char *end = NULL;
    if (text == NULL || *text == '\0') return false;
    double value = strtod(text, &end);
    if (*end != '\0' || !isfinite(value) || fabs(value) > limit) return false;
    *out = (int32_t)lround(value * 1e6);
    return true;
}

/* Splits at commas without skipping empty fields (strtok would). */
static unsigned split(char *text, char **fields, unsigned max)
{
    unsigned count = 0U;
    fields[count++] = text;
    for (char *p = text; *p != '\0' && count < max; p++) {
        if (*p == ',') {
            *p = '\0';
            fields[count++] = p + 1;
        }
    }
    return count;
}

bool RouteLink_OnFrame(route_link_t *route, const char *frame, const char *buoy_id)
{
    char copy[MAX_FRAME];
    size_t length = strlen(frame);
    if (length < 7U || length >= sizeof(copy) || strncmp(frame, "RTE,", 4U) != 0) return false;
    /* "...*CS": XOR of everything before the '*'. */
    const char *star = frame + length - 3U;
    uint8_t cs = 0U;
    for (const char *p = frame; p < star; p++) cs ^= (uint8_t)*p;
    char expected[4];
    snprintf(expected, sizeof(expected), "*%02X", cs);
    if (strcmp(star, expected) != 0) { route->frames_bad++; return false; }
    length -= 3U;
    memcpy(copy, frame, length);
    copy[length] = '\0';
    char *fields[6U + 2U * 8U];
    unsigned count = split(copy, fields, sizeof(fields) / sizeof(fields[0]));
    if (count < 6U || strcmp(fields[1], buoy_id) != 0) return false;

    unsigned long rev, first, total, loop;
    unsigned points = (count - 6U) / 2U;
    if (!parse_uint(fields[2], 0xFFFFUL, &rev) || rev == 0UL ||
        !parse_uint(fields[3], ROUTE_MAX_POINTS, &first) ||
        !parse_uint(fields[4], ROUTE_MAX_POINTS, &total) ||
        !parse_uint(fields[5], 1UL, &loop) ||
        (count - 6U) % 2U != 0U || first + points > total) {
        route->frames_bad++;
        return false;
    }
    route_point_t parsed[8];
    for (unsigned i = 0U; i < points; i++) {
        if (!parse_coord(fields[6U + 2U * i], 90.0, &parsed[i].lat_e6) ||
            !parse_coord(fields[7U + 2U * i], 180.0, &parsed[i].lon_e6)) {
            route->frames_bad++;
            return false;
        }
    }
    if ((uint16_t)rev != route->rev) {
        /* A new route starts from its first point; a stray later frame of it waits for the resend. */
        if (first != 0UL) { route->frames_bad++; return false; }
        route->rev = (uint16_t)rev;
        route->total = (uint16_t)total;
        route->loop = loop != 0UL;
        route->have = 0U;
        route->pi_next = 0U;
        route->pi_paused = false;
    } else if (first != route->have || total != route->total) {
        route->frames_bad++;   /* a repeat of a frame already taken, or out of order */
        return false;
    }
    memcpy(&route->points[route->have], parsed, points * sizeof(parsed[0]));
    route->have = (uint16_t)(route->have + points);
    route->frames_ok++;
    return true;
}

size_t RouteLink_BeaconField(const route_link_t *route, char *buf, size_t size)
{
    if (size == 0U) return 0U;
    buf[0] = '\0';
    if (route->rev == 0U) return 0U;
    int n = snprintf(buf, size, "w=%u:%u:%u", (unsigned)route->rev, (unsigned)route->have,
                     RouteLink_Complete(route) && route->pi_rev == route->rev ? 1U : 0U);
    if (n < 0 || (size_t)n >= size) { buf[0] = '\0'; return 0U; }
    return (size_t)n;
}

void RouteLink_SetPiRev(route_link_t *route, uint16_t rev)
{
    route->pi_rev = rev;
}

/* "10.883438" from 10883438; the sign is written separately so -0.5 keeps its minus. */
static int format_e6(char *buf, size_t size, int32_t value)
{
    uint32_t magnitude = value < 0 ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
    return snprintf(buf, size, "%s%lu.%06lu", value < 0 ? "-" : "", (unsigned long)(magnitude / 1000000UL),
                    (unsigned long)(magnitude % 1000000UL));
}

bool RouteLink_NextPiLine(route_link_t *route, uint16_t seq, uint32_t now_ms, char *buf, size_t size)
{
    if (!RouteLink_Complete(route) || route->pi_rev == route->rev) {
        route->pi_next = 0U;
        route->pi_paused = false;
        return false;
    }
    if (route->pi_paused) {
        if ((int32_t)(now_ms - route->pi_pause_until_ms) < 0) return false;
        route->pi_paused = false;
        route->pi_next = 0U;
    }
    int n = snprintf(buf, size, "%u,%u,%u,%u,%u", (unsigned)seq, (unsigned)route->rev, (unsigned)route->pi_next,
                     (unsigned)route->total, route->loop ? 1U : 0U);
    if (n < 0 || (size_t)n >= size) return false;
    size_t used = (size_t)n;
    uint16_t index = route->pi_next;
    for (unsigned i = 0U; i < ROUTE_PI_POINTS_PER_LINE && index < route->total; i++, index++) {
        char lat[16], lon[16];
        format_e6(lat, sizeof(lat), route->points[index].lat_e6);
        format_e6(lon, sizeof(lon), route->points[index].lon_e6);
        n = snprintf(buf + used, size - used, ",%s;%s", lat, lon);
        if (n < 0 || (size_t)n >= size - used) return false;
        used += (size_t)n;
    }
    route->pi_next = index;
    if (route->pi_next >= route->total) {
        route->pi_paused = true;
        route->pi_pause_until_ms = now_ms + ROUTE_PI_RESEND_MS;
    }
    return true;
}
