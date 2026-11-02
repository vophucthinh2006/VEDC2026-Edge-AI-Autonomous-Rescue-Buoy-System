#include "shore_link.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_FRAME 128U
#define ACK_FIELDS 6U
#define ACK_FIELDS_FORMATION 9U
#define ACK_FIELDS_SKIP 11U
#define FORMATION_MAX_M 500L

void ShoreLink_Init(shore_link_t *shore)
{
    memset(shore, 0, sizeof(*shore));
}

void ShoreLink_Heard(shore_link_t *shore, uint32_t now_ms)
{
    shore->ever_heard = true;
    shore->last_heard_ms = now_ms;
}

static bool parse_uint(const char *text, unsigned long max, unsigned long *out)
{
    char *end = NULL;
    if (*text < '0' || *text > '9') return false;
    unsigned long value = strtoul(text, &end, 10);
    if (*end != '\0' || value > max) return false;
    *out = value;
    return true;
}

/* "-12" or "7": a whole number of metres, |value| <= limit. */
static bool parse_int(const char *text, long limit, long *out)
{
    char *end = NULL;
    const char *digits = *text == '-' ? text + 1 : text;
    if (*digits < '0' || *digits > '9') return false;
    long value = strtol(text, &end, 10);
    if (*end != '\0' || value > limit || value < -limit) return false;
    *out = value;
    return true;
}

static bool parse_coord(const char *text, double limit, int32_t *out)
{
    char *end = NULL;
    double value = strtod(text, &end);
    if (*text == '\0' || *end != '\0' || !isfinite(value) || fabs(value) > limit) return false;
    *out = (int32_t)lround(value * 1e6);
    return true;
}

bool ShoreLink_OnFrame(shore_link_t *shore, const char *frame, const char *buoy_id, uint32_t now_ms, uint16_t *eseq)
{
    char copy[MAX_FRAME];
    size_t length = strlen(frame);
    if (length < 7U || length >= sizeof(copy) || strncmp(frame, "ACK,", 4U) != 0) return false;
    /* "...*CS": XOR of everything before the '*', as in RTE frames (App/route_link.c). */
    const char *star = frame + length - 3U;
    uint8_t cs = 0U;
    for (const char *p = frame; p < star; p++) cs ^= (uint8_t)*p;
    char expected[4];
    snprintf(expected, sizeof(expected), "*%02X", cs);
    if (strcmp(star, expected) != 0) { shore->acks_bad++; return false; }
    length -= 3U;
    memcpy(copy, frame, length);
    copy[length] = '\0';

    /* Split at commas, keeping empty fields: lat and lon are empty while the station has no fix. */
    char *fields[ACK_FIELDS_SKIP];
    unsigned count = 0U;
    fields[count++] = copy;
    for (char *p = copy; *p != '\0'; p++) {
        if (*p != ',') continue;
        if (count == ACK_FIELDS_SKIP) { shore->acks_bad++; return false; }
        *p = '\0';
        fields[count++] = p + 1;
    }
    if (count != ACK_FIELDS && count != ACK_FIELDS_FORMATION && count != ACK_FIELDS_SKIP) { shore->acks_bad++; return false; }
    if (strcmp(fields[1], buoy_id) != 0) return false;

    unsigned long acked, lights;
    int32_t lat = 0, lon = 0;
    bool has_home = fields[4][0] != '\0' || fields[5][0] != '\0';
    long right = 0L, back = 0L;
    /* Eleven fields: the formation ones may be empty (no formation), then the person to leave alone. */
    bool skip = count == ACK_FIELDS_SKIP;
    bool follow = count == ACK_FIELDS_FORMATION || (skip && fields[6][0] != '\0');
    bool no_role = skip && !follow && fields[7][0] == '\0' && fields[8][0] == '\0';
    int32_t skip_lat = 0, skip_lon = 0;
    if (!parse_uint(fields[2], 0xFFFFUL, &acked) || !parse_uint(fields[3], 15UL, &lights) ||
        (has_home && (!parse_coord(fields[4], 90.0, &lat) || !parse_coord(fields[5], 180.0, &lon))) ||
        (count >= ACK_FIELDS_FORMATION && !follow && !no_role) ||
        (follow && (fields[6][0] == '\0' || strlen(fields[6]) >= sizeof(shore->leader) ||
                    !parse_int(fields[7], FORMATION_MAX_M, &right) || !parse_int(fields[8], FORMATION_MAX_M, &back))) ||
        (skip && (!parse_coord(fields[9], 90.0, &skip_lat) || !parse_coord(fields[10], 180.0, &skip_lon)))) {
        shore->acks_bad++;
        return false;
    }
    ShoreLink_Heard(shore, now_ms);
    shore->lights = (uint8_t)(lights & 0x03UL);
    shore->hold = (lights & 0x04UL) != 0UL;
    shore->ret = (lights & 0x08UL) != 0UL;
    shore->follow = follow;
    if (follow) {
        strcpy(shore->leader, fields[6]);
        shore->right_m = (int16_t)right;
        shore->back_m = (int16_t)back;
    } else {
        shore->leader[0] = '\0';
        shore->right_m = 0;
        shore->back_m = 0;
    }
    shore->lights_known = true;
    if (skip) {
        shore->skip_new = true;
        shore->skip_lat_e6 = skip_lat;
        shore->skip_lon_e6 = skip_lon;
    }
    if (has_home) {
        /* A station without a fix keeps the last home it gave. */
        shore->home_valid = true;
        shore->home_lat_e6 = lat;
        shore->home_lon_e6 = lon;
    }
    shore->acks_ok++;
    *eseq = (uint16_t)acked;
    return true;
}

bool ShoreLink_Ok(const shore_link_t *shore, uint32_t now_ms)
{
    return !shore->ever_heard || (uint32_t)(now_ms - shore->last_heard_ms) < SHORE_LINK_TIMEOUT_MS;
}

/* "10.883438" from 10883438; the sign is written separately so -0.5 keeps its minus. */
static int format_e6(char *buf, size_t size, int32_t value)
{
    uint32_t magnitude = value < 0 ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
    return snprintf(buf, size, "%s%lu.%06lu", value < 0 ? "-" : "", (unsigned long)(magnitude / 1000000UL),
                    (unsigned long)(magnitude % 1000000UL));
}

size_t ShoreLink_PiLine(const shore_link_t *shore, uint16_t seq, uint32_t now_ms, char *buf, size_t size)
{
    char ago[16], lat[16] = "", lon[16] = "";
    if (shore->ever_heard) {
        uint32_t ms = now_ms - shore->last_heard_ms;
        snprintf(ago, sizeof(ago), "%lu.%lu", (unsigned long)(ms / 1000UL), (unsigned long)(ms % 1000UL / 100UL));
    } else {
        strcpy(ago, "-1");
    }
    if (shore->home_valid) {
        format_e6(lat, sizeof(lat), shore->home_lat_e6);
        format_e6(lon, sizeof(lon), shore->home_lon_e6);
    }
    int n = snprintf(buf, size, "%u,%u,%s,%u,%s,%s,%u,%u", (unsigned)seq, ShoreLink_Ok(shore, now_ms) ? 1U : 0U, ago,
                     shore->home_valid ? 1U : 0U, lat, lon, shore->hold ? 1U : 0U, shore->ret ? 1U : 0U);
    if (n < 0 || (size_t)n >= size) return 0U;
    return (size_t)n;
}

size_t ShoreLink_FollowLine(const shore_link_t *shore, uint16_t seq, char *buf, size_t size)
{
    int n = shore->follow
          ? snprintf(buf, size, "%u,%s,%d,%d", (unsigned)seq, shore->leader, shore->right_m, shore->back_m)
          : snprintf(buf, size, "%u,,,", (unsigned)seq);
    if (n < 0 || (size_t)n >= size) return 0U;
    return (size_t)n;
}

size_t ShoreLink_SkipLine(shore_link_t *shore, uint16_t seq, char *buf, size_t size)
{
    if (!shore->skip_new) return 0U;
    char lat[16], lon[16];
    format_e6(lat, sizeof(lat), shore->skip_lat_e6);
    format_e6(lon, sizeof(lon), shore->skip_lon_e6);
    int n = snprintf(buf, size, "%u,%s,%s", (unsigned)seq, lat, lon);
    if (n < 0 || (size_t)n >= size) return 0U;
    shore->skip_new = false;
    return (size_t)n;
}
