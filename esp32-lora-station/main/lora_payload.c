#include "lora_payload.h"
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static bool parse_float(const char *text, float *out) {
    char *end = NULL;
    errno = 0;
    float value = strtof(text, &end);
    if (end == text || *end != '\0' || errno == ERANGE || !isfinite(value)) return false;
    *out = value;
    return true;
}

static bool parse_uint(const char *text, uint32_t *out) {
    char *end = NULL;
    errno = 0;
    unsigned long value = strtoul(text, &end, 10);
    if (end == text || *end != '\0' || errno == ERANGE || value > UINT32_MAX) return false;
    *out = (uint32_t)value;
    return true;
}

bool lora_payload_parse(const char *message, lora_payload_t *out) {
    if (message == NULL || out == NULL) return false;
    memset(out, 0, sizeof(*out));
    out->target_yaw = -1.0f;
    out->mode = 'S';

    char buffer[128];
    size_t length = strlen(message);
    if (length == 0U || length >= sizeof(buffer)) return false;
    memcpy(buffer, message, length + 1U);

    bool found_r = false, found_p = false, found_y = false, found_t = false;
    bool found_m = false, found_i = false, found_c = false, found_q = false;
    bool saw_attitude = false;
    double coordinates[2];
    unsigned int coordinate_count = 0U;

    for (char *token = strtok(buffer, ","); token != NULL; token = strtok(NULL, ",")) {
        while (*token == ' ') token++;
        char *equals = strchr(token, '=');
        if (equals == NULL) {
            if (strchr(token, '.') != NULL && coordinate_count < 2U) {
                char *end = NULL;
                double value = strtod(token, &end);
                if (end != token && *end == '\0' && isfinite(value)) coordinates[coordinate_count++] = value;
            }
            continue;
        }

        *equals = '\0';
        const char *value = equals + 1;
        if (strcmp(token, "id") == 0) {
            strncpy(out->id, value, sizeof(out->id) - 1U);
        } else if (strcmp(token, "r") == 0) {
            saw_attitude = true;
            found_r = parse_float(value, &out->roll);
        } else if (strcmp(token, "p") == 0) {
            saw_attitude = true;
            found_p = parse_float(value, &out->pitch);
        } else if (strcmp(token, "y") == 0) {
            saw_attitude = true;
            found_y = parse_float(value, &out->yaw);
        } else if (strcmp(token, "t") == 0) {
            saw_attitude = true;
            found_t = parse_float(value, &out->target_yaw);
        } else if (strcmp(token, "m") == 0) {
            saw_attitude = true;
            found_m = value[0] != '\0' && value[1] == '\0';
            if (found_m) out->mode = value[0];
        } else if (strcmp(token, "i") == 0) {
            saw_attitude = true;
            uint32_t parsed;
            found_i = parse_uint(value, &parsed) && parsed <= 1U;
            if (found_i) out->imu_ok = (uint8_t)parsed;
        } else if (strcmp(token, "c") == 0) {
            saw_attitude = true;
            uint32_t parsed;
            found_c = parse_uint(value, &parsed) && parsed <= 3U;
            if (found_c) out->calibration = (uint8_t)parsed;
        } else if (strcmp(token, "q") == 0) {
            saw_attitude = true;
            found_q = parse_uint(value, &out->sequence);
        } else if (strcmp(token, "v") == 0) {
            /* Not part of the attitude group: a frame without it is still complete. */
            uint32_t parsed;
            out->has_victims = parse_uint(value, &parsed) && parsed <= 255U;
            if (out->has_victims) out->victims = (uint8_t)parsed;
        } else if (strcmp(token, "w") == 0) {
            /* Route progress (rev:have:pi), outside the attitude group too. */
            char *first_colon = strchr(value, ':');
            char *second_colon = first_colon != NULL ? strchr(first_colon + 1, ':') : NULL;
            uint32_t rev = 0U, have = 0U, pi = 0U;
            if (second_colon != NULL) {
                *first_colon = '\0';
                *second_colon = '\0';
                out->has_route = parse_uint(value, &rev) && rev >= 1U && rev <= 0xFFFFU &&
                                 parse_uint(first_colon + 1, &have) && have <= 300U &&
                                 parse_uint(second_colon + 1, &pi) && pi <= 1U;
            }
            if (out->has_route) {
                out->route_rev = (uint16_t)rev;
                out->route_have = (uint16_t)have;
                out->route_pi = (uint8_t)pi;
            }
        } else if (strcmp(token, "g") == 0) {
            out->has_course = parse_float(value, &out->course) && out->course >= 0.0f && out->course < 360.0f;
        } else if (strcmp(token, "s") == 0) {
            out->has_speed = parse_float(value, &out->speed) && out->speed >= 0.0f && out->speed < 50.0f;
        } else if (strcmp(token, "b") == 0) {
            uint32_t parsed;
            out->has_battery = parse_uint(value, &parsed) && parsed <= 100U;
            if (out->has_battery) out->battery_pct = (uint8_t)parsed;
        } else if (strcmp(token, "H") == 0) {
            uint32_t parsed;
            out->has_hold = parse_uint(value, &parsed) && parsed <= 1U;
            if (out->has_hold) out->hold = (uint8_t)parsed;
        } else if (strcmp(token, "L") == 0) {
            uint32_t parsed;
            out->has_lights = parse_uint(value, &parsed) && parsed <= 3U;
            if (out->has_lights) out->lights = (uint8_t)parsed;
        }
    }

    if (coordinate_count == 2U && fabs(coordinates[0]) <= 90.0 && fabs(coordinates[1]) <= 180.0) {
        out->has_position = true;
        out->lat = coordinates[0];
        out->lon = coordinates[1];
    }

    if (!saw_attitude) return true;

    out->has_attitude = found_r && found_p && found_y && found_t && found_m && found_i && found_c && found_q &&
                        fabsf(out->roll) <= 180.0f && fabsf(out->pitch) <= 90.0f &&
                        out->yaw >= 0.0f && out->yaw < 360.0f &&
                        (out->target_yaw == -1.0f || (out->target_yaw >= 0.0f && out->target_yaw < 360.0f)) &&
                        (out->mode == 'A' || out->mode == 'M' || out->mode == 'S' || out->mode == 'R');
    /* A bad attitude group costs only the attitude: the position of a rescue
       buoy is worth more than one corrupt IMU field. */
    out->attitude_rejected = !out->has_attitude;
    return true;
}

static bool parse_double(const char *text, double limit, double *out) {
    char *end = NULL;
    errno = 0;
    double value = strtod(text, &end);
    if (end == text || *end != '\0' || errno == ERANGE || !isfinite(value) || fabs(value) > limit) return false;
    *out = value;
    return true;
}

bool lora_event_parse(const char *message, lora_event_t *out) {
    if (message == NULL || out == NULL) return false;
    memset(out, 0, sizeof(*out));
    char buffer[128];
    size_t length = strlen(message);
    if (length < 4U || length >= sizeof(buffer) || strncmp(message, "ev=", 3U) != 0) return false;
    memcpy(buffer, message + 3, length - 2U);

    /* Split at commas keeping empty fields: lat and lon are empty without a position. */
    char *fields[8];
    unsigned count = 0U;
    fields[count++] = buffer;
    for (char *p = buffer; *p != '\0'; p++) {
        if (*p != ',') continue;
        if (count == 8U) return false;
        *p = '\0';
        fields[count++] = p + 1;
    }
    if (count < 6U || fields[0][0] == '\0' || strlen(fields[0]) >= sizeof(out->id)) return false;
    strcpy(out->id, fields[0]);
    uint32_t seq;
    if (!parse_uint(fields[1], &seq) || seq == 0U || seq > 0xFFFFU) return false;
    out->seq = (uint16_t)seq;
    if (fields[2][0] == '\0' || fields[2][1] != '\0') return false;
    out->type = fields[2][0];
    if (fields[3][0] != '\0' || fields[4][0] != '\0') {
        if (!parse_double(fields[3], 90.0, &out->lat) || !parse_double(fields[4], 180.0, &out->lon)) return false;
        out->has_position = true;
    }
    uint32_t parsed;
    switch (out->type) {
    case 'V':
        if (count != 8U || !parse_uint(fields[5], &parsed) || parsed > 3U) return false;
        out->risk = (uint8_t)parsed;
        if (!parse_float(fields[6], &out->confidence) || out->confidence < 0.0f || out->confidence > 1.0f) return false;
        if (!parse_uint(fields[7], &parsed) || parsed > 0xFFFFU) return false;
        out->count = (uint16_t)parsed;
        return true;
    case 'T':
        return count == 7U && parse_float(fields[5], &out->roll) && fabsf(out->roll) <= 180.0f &&
               parse_float(fields[6], &out->pitch) && fabsf(out->pitch) <= 180.0f;
    case 'C':
        if (count != 6U || !parse_uint(fields[5], &parsed) || parsed > 0xFFFFU) return false;
        out->presses = (uint16_t)parsed;
        return true;
    default:
        return false;
    }
}
