#include "gps_nmea.h"
#include <stdlib.h>
#include <string.h>

static float ddmm_to_deg(const char *value, char hemisphere) {
    float ddmm = strtof(value, NULL);
    int degree = (int)(ddmm / 100.0f);
    float decimal = (float)degree + (ddmm - (float)degree * 100.0f) / 60.0f;
    return (hemisphere == 'S' || hemisphere == 'W') ? -decimal : decimal;
}

void GPS_Init(gps_state_t *state) { memset(state, 0, sizeof(*state)); }

void GPS_FeedByte(gps_state_t *state, uint8_t byte, uint32_t now_ms) {
    static char line[128];
    static uint8_t length = 0U;
    if (byte == '\r') return;
    if (byte != '\n') {
        if (length < sizeof(line) - 1U) line[length++] = (char)byte;
        else length = 0U;
        return;
    }
    line[length] = '\0';
    length = 0U;
    if (strncmp(line, "$GNGGA,", 7U) != 0 && strncmp(line, "$GPGGA,", 7U) != 0) return;
    char *fields[12] = {0};
    uint8_t count = 0U;
    char *token = strtok(line, ",");
    while (token && count < 12U) { fields[count++] = token; token = strtok(NULL, ","); }
    if (count < 9U) return;
    state->latitude_deg = ddmm_to_deg(fields[2], fields[3][0]);
    state->longitude_deg = ddmm_to_deg(fields[4], fields[5][0]);
    state->fix = (uint8_t)strtoul(fields[6], NULL, 10);
    state->hdop = strtof(fields[8], NULL);
    state->last_rx_ms = now_ms;
    state->valid = state->fix > 0U;
}
