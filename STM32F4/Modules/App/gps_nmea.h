#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float latitude_deg;
    float longitude_deg;
    float hdop;
    float speed_mps;
    float course_deg;
    uint8_t fix;
    uint32_t last_rx_ms;
    bool valid;
} gps_state_t;

void GPS_Init(gps_state_t *state);
void GPS_FeedByte(gps_state_t *state, uint8_t byte, uint32_t now_ms);
