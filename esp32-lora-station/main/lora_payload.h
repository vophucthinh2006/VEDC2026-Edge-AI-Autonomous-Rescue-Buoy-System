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
    bool has_route;           /* w=rev:have:pi present and valid (docs/lora-route-downlink.md) */
    uint16_t route_rev;       /* route the buoy holds or is receiving */
    uint16_t route_have;      /* points received in order from the first */
    uint8_t route_pi;         /* 1: the Pi confirmed route_rev */
    /* Optional, outside the attitude group; the buoy leaves out what does not fit or is unknown. */
    bool has_course;          /* g=: GPS course over ground, the way the boat really goes */
    float course;
    bool has_speed;           /* s=: GPS speed, m/s */
    float speed;
    bool has_battery;         /* b=: battery charge, % */
    uint8_t battery_pct;
    bool has_lights;          /* L=: relays the buoy drives, bit 0 light 1 (PC3), bit 1 light 2 (PC2) */
    uint8_t lights;
    bool has_hold;            /* H=: 1 while stopped from the dashboard (motors off) */
    uint8_t hold;
} lora_payload_t;

/* An event frame of the buoy, sent until the station acknowledges it (STM32F4/App/event_queue.h):
     ev=PHAO-01,<eseq>,V,<lat>,<lon>,<risk>,<conf>,<n>     a person
     ev=PHAO-01,<eseq>,T,<lat>,<lon>,<roll>,<pitch>         overturned, the boat's GPS
     ev=PHAO-01,<eseq>,C,<lat>,<lon>,<presses>              confirm button: supplies taken
   lat and lon are empty without a position. */
typedef struct {
    char id[32];
    uint16_t seq;
    char type;                /* 'V', 'T' or 'C' */
    bool has_position;
    double lat;
    double lon;
    uint8_t risk;             /* V: 0 safe, 1 medium, 2 high, 3 critical */
    float confidence;         /* V: 0..1 */
    uint16_t count;           /* V: people the buoy has reported */
    float roll;               /* T */
    float pitch;              /* T */
    uint16_t presses;         /* C */
} lora_event_t;

/* True only for a complete, in-range event frame. */
bool lora_event_parse(const char *message, lora_event_t *out);

/* Parses both the legacy GPS-only frame and the attitude-extended frame.
   The attitude group counts only when complete and in range; otherwise
   has_attitude is false and attitude_rejected true, and the position (if
   any) is still returned. False only for an empty or oversized message. */
bool lora_payload_parse(const char *message, lora_payload_t *out);
