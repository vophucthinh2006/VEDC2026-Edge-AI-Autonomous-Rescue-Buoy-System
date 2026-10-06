#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    char id[32];
    bool has_position;
    double lat;
    double lon;
    bool has_attitude;
    bool attitude_rejected;   /* attitude fields were present but incomplete or out of range */
    float roll;
    float pitch;
    float yaw;
    float target_yaw;
    char mode;
    uint8_t imu_ok;
    uint8_t calibration;
    uint32_t sequence;
    bool has_victims;         /* v= present and in range; older buoy firmware does not send it */
    uint8_t victims;          /* people the buoy has reported since it booted */
} lora_payload_t;

/* Parses both the legacy GPS-only frame and the attitude-extended frame.
   The attitude group counts only when complete and in range; otherwise
   has_attitude is false and attitude_rejected true, and the position (if
   any) is still returned. False only for an empty or oversized message. */
bool lora_payload_parse(const char *message, lora_payload_t *out);
