#include "pi_link.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_FIELDS 12U

static uint8_t xor_checksum(const char *text) { uint8_t cs = 0U; while (*text) cs ^= (uint8_t)*text++; return cs; }

/* A plain decimal number, as the Pi prints them: -12.5, 106.660172, 3. */
static bool is_number(const char *text, size_t max_length) {
    size_t length = strlen(text);
    bool digit = false, dot = false;
    if (length == 0U || length > max_length) return false;
    for (size_t i = 0U; i < length; i++) {
        char c = text[i];
        if (c >= '0' && c <= '9') digit = true;
        else if (c == '.' && !dot) dot = true;
        else if (c == '-' && i == 0U) continue;
        else return false;
    }
    return digit;
}

/* "-12.34" to -123: tenths, the hundredths dropped. The text has passed is_number. */
static int32_t tenths(const char *text) {
    bool negative = *text == '-';
    if (negative) text++;
    int32_t value = 0;
    while (*text >= '0' && *text <= '9' && value < 100000) value = value * 10 + (*text++ - '0');
    value *= 10;
    if (*text == '.' && text[1] >= '0' && text[1] <= '9') value += text[1] - '0';
    return negative ? -value : value;
}

void PiLink_Init(pi_link_t *link) { memset(link, 0, sizeof(*link)); }

static bool handle_status(pi_link_t *link, char **f, uint8_t count) {
    /* f: POS seq fix lat lon roll pitch yaw target mode imu_ok calib */
    if (count < 12U) return false;
    bool fix = strcmp(f[2], "1") == 0;
    if (!fix && strcmp(f[2], "0") != 0) return false;
    if (!is_number(f[3], 11U) || !is_number(f[4], 11U) || !is_number(f[5], 6U) || !is_number(f[6], 5U) ||
        !is_number(f[7], 5U) || !is_number(f[8], 5U)) return false;
    if (strlen(f[9]) != 1U || strchr("AMS", f[9][0]) == NULL) return false;
    if (strlen(f[10]) != 1U || (f[10][0] != '0' && f[10][0] != '1')) return false;
    if (strlen(f[11]) != 1U || f[11][0] < '0' || f[11][0] > '3') return false;
    char status[PI_LINK_STATUS_MAX];
    int n = fix ? snprintf(status, sizeof(status), "%s,%s,r=%s,p=%s,y=%s,t=%s,m=%s,i=%s,c=%s", f[3], f[4], f[5], f[6], f[7], f[8], f[9], f[10], f[11])
                : snprintf(status, sizeof(status), "NO_FIX,r=%s,p=%s,y=%s,t=%s,m=%s,i=%s,c=%s", f[5], f[6], f[7], f[8], f[9], f[10], f[11]);
    if (n < 0 || (size_t)n >= sizeof(status)) return false;
    memcpy(link->status, status, (size_t)n + 1U);
    return true;
}

static bool handle_packet(pi_link_t *link, char *payload, uint32_t now_ms) {
    char *fields[MAX_FIELDS] = {0};
    uint8_t count = 0U;
    char *token = strtok(payload, ",");
    while (token && count < MAX_FIELDS) { fields[count++] = token; token = strtok(NULL, ","); }
    if (count < 2U) return false;
    if (strcmp(fields[0], "HBT") == 0) {
        link->seen = true;
        link->last_hbt_ms = now_ms;
    } else if (strcmp(fields[0], "CAM") == 0 && count >= 3U) {
        if (!is_number(fields[2], 6U)) return false;
        int32_t pan = tenths(fields[2]);
        if (pan < -900 || pan > 900) return false;
        link->cam_pan_ddeg = (int16_t)pan;
        link->cam_seen = true;
        link->last_cam_ms = now_ms;
    } else if (strcmp(fields[0], "VIC") == 0 && count >= 3U) {
        if (!is_number(fields[2], 3U) || fields[2][0] == '-') return false;
        unsigned long victims = strtoul(fields[2], NULL, 10);
        if (victims > 255UL) return false;
        link->victims = (uint8_t)victims;
    } else if (strcmp(fields[0], "POS") == 0) {
        if (!handle_status(link, fields, count)) return false;
        link->status_seen = true;
        link->last_status_ms = now_ms;
    } else {
        return false;
    }
    return true;
}

void PiLink_FeedByte(pi_link_t *link, uint8_t byte, uint32_t now_ms) {
    if (byte == '\r') return;
    if (byte != '\n') {
        if (link->length < PI_LINK_MAX_LINE - 1U) link->line[link->length++] = (char)byte;
        else link->length = 0U;
        return;
    }
    link->line[link->length] = '\0';
    link->length = 0U;
    if (link->line[0] == '\0') return;
    bool ok = false;
    char *star = strchr(link->line, '*');
    if (link->line[0] == '$' && star != NULL && strlen(star + 1) == 2U) {
        char *end = NULL;
        unsigned long received = strtoul(star + 1, &end, 16);
        *star = '\0';
        if (end != NULL && *end == '\0' && xor_checksum(link->line + 1) == (uint8_t)received)
            ok = handle_packet(link, link->line + 1, now_ms);
    }
    if (ok) link->rx_ok++; else link->rx_bad++;
}

size_t PiLink_Encode(char *out, size_t size, const char *command, const char *payload) {
    char body[PI_LINK_MAX_LINE];
    int body_size = snprintf(body, sizeof(body), "%s,%s", command, payload);
    if (body_size < 0 || (size_t)body_size >= sizeof(body)) return 0U;
    int line_size = snprintf(out, size, "$%s*%02X\n", body, xor_checksum(body));
    if (line_size < 0 || (size_t)line_size >= size) return 0U;
    return (size_t)line_size;
}
