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
                        (out->mode == 'A' || out->mode == 'M' || out->mode == 'S');
    /* A bad attitude group costs only the attitude: the position of a rescue
       buoy is worth more than one corrupt IMU field. */
    out->attitude_rejected = !out->has_attitude;
    return true;
}
