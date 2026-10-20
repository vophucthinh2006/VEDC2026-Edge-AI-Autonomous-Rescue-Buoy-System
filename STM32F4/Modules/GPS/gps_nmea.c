#include "gps_nmea.h"
#include <stdlib.h>
#include <string.h>

#define MAX_FIELDS 20U
#define KNOTS_TO_MPS 0.514444f

static float ddmm_to_deg(const char *value, char hemisphere) {
    float ddmm = strtof(value, NULL);
    int degree = (int)(ddmm / 100.0f);
    float decimal = (float)degree + (ddmm - (float)degree * 100.0f) / 60.0f;
    return (hemisphere == 'S' || hemisphere == 'W') ? -decimal : decimal;
}

static int hex_digit(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/* "$...*hh": XOR of everything between '$' and '*'. A sentence without '*' is taken as it is. */
static bool checksum_ok(char *line) {
    char *star = strchr(line, '*');
    if (star == NULL) return true;
    int high = hex_digit(star[1]), low = star[1] != '\0' ? hex_digit(star[2]) : -1;
    if (high < 0 || low < 0) return false;
    unsigned cs = 0U;
    for (const char *p = line + 1; p < star; p++) cs ^= (unsigned char)*p;
    *star = '\0';
    return cs == (unsigned)(high * 16 + low);
}

/* Splits at commas keeping empty fields: a GGA without a fix is mostly empty, and strtok would
   shift every field after the first gap. */
static unsigned split(char *text, char **fields) {
    unsigned count = 0U;
    fields[count++] = text;
    for (char *p = text; *p != '\0' && count < MAX_FIELDS; p++) {
        if (*p == ',') {
            *p = '\0';
            fields[count++] = p + 1;
        }
    }
    return count;
}

void GPS_Init(gps_state_t *state) { memset(state, 0, sizeof(*state)); }

void GPS_FeedByte(gps_state_t *state, uint8_t byte, uint32_t now_ms) {
    static char line[128];
    static uint8_t length = 0U;
    state->bytes++;
    if (byte == '\r') return;
    if (byte == '$') length = 0U;              /* a new sentence, even after a cut one */
    if (byte != '\n') {
        if (length < sizeof(line) - 1U) line[length++] = (char)byte;
        else length = 0U;
        return;
    }
    line[length] = '\0';
    length = 0U;
    if (line[0] != '$' || (line[1] != 'G' || (line[2] != 'P' && line[2] != 'N'))) return;
    bool gga = strncmp(line + 3, "GGA,", 4U) == 0, rmc = strncmp(line + 3, "RMC,", 4U) == 0;
    if (!gga && !rmc) return;
    if (!checksum_ok(line)) { state->bad_checksum++; return; }
    char *fields[MAX_FIELDS];
    unsigned count = split(line, fields);
    if (gga) {
        /* $GxGGA,time,lat,N,lon,E,quality,sats,hdop,alt,M,... */
        if (count < 9U) return;
        state->fix = (uint8_t)strtoul(fields[6], NULL, 10);
        if (state->fix > 0U && fields[2][0] != '\0' && fields[4][0] != '\0') {
            state->latitude_deg = ddmm_to_deg(fields[2], fields[3][0]);
            state->longitude_deg = ddmm_to_deg(fields[4], fields[5][0]);
        }
        state->hdop = fields[8][0] != '\0' ? strtof(fields[8], NULL) : 99.0f;
        state->last_rx_ms = now_ms;
        state->valid = state->fix > 0U;
    } else {
        /* $GxRMC,time,status,lat,N,lon,E,speed_knots,course,date,... ; status A = valid */
        if (count < 9U || fields[2][0] != 'A') return;
        state->speed_mps = strtof(fields[7], NULL) * KNOTS_TO_MPS;
        /* Course is empty while standing still: keep the last one. */
        if (fields[8][0] != '\0') state->course_deg = strtof(fields[8], NULL);
    }
    state->sentences++;
}
