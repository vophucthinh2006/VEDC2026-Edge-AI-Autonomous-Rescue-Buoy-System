#pragma once

#include <stdbool.h>
#include <stdint.h>

/* NMEA from the GPS (USART3): GGA for the fix, position and HDOP, RMC for the speed and the course
 * over ground (the way the boat really goes, against the bow's heading from the IMU). Talker GP or
 * GN; a sentence with a wrong *hh checksum is dropped. Plain C, no HAL: host test in Tests/.
 * Tools/lora_monitor.py reads this struct over ST-LINK: append new fields at the end. */
typedef struct {
    float latitude_deg;
    float longitude_deg;
    float hdop;
    float speed_mps;
    float course_deg;            /* 0..360, valid while moving (RMC) */
    uint8_t fix;                 /* GGA fix quality, 0 = none */
    uint32_t last_rx_ms;         /* last GGA, fix or not: the GPS is talking */
    bool valid;                  /* the last GGA had a fix */
    uint32_t bytes;              /* bytes received since boot: 0 = wiring or baud */
    uint32_t sentences;          /* GGA and RMC taken since boot */
    uint32_t bad_checksum;
} gps_state_t;

void GPS_Init(gps_state_t *state);
void GPS_FeedByte(gps_state_t *state, uint8_t byte, uint32_t now_ms);
