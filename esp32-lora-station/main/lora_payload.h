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

/* The buoy's diagnostics frame (STM32F4/App/diag_frame.h), every 10 s between beacons, not answered:
     dg=PHAO-01,n=obstacle_too_close,s=P,w=2/5,a=0,d=0,u=1184,j=-35,k=0:0
   n the Pi's NAV reason ("-" unknown), s its mission (I P F R B H, "-" unknown), w waypoint/total,
   a arm block, d drive block, u volts x100 and j amps x100 (both or none), k I2C recoveries:IMU re-inits,
   f the Pi's detector frames a second x10 (optional: left out while the detector is off). */
typedef struct {
    char id[32];
    char reason[24];          /* "" unknown */
    char mission;             /* 0 unknown */
    uint16_t wp;
    uint16_t wp_total;
    uint8_t arm_block;        /* 0 armed, 1 no fresh IMU, 2 overturned */
    uint8_t drive_block;      /* 0 driving, 1 disarmed, 2 no NAV from the Pi, 3 IMU stale, 4 STOP from the dashboard */
    bool has_battery;
    float volts;
    float amps;
    uint16_t i2c_recoveries;
    uint16_t imu_reinits;
    bool has_fps;
    float fps;                /* the Pi's detector frames a second */
} lora_diag_t;

/* True only for a complete, in-range diagnostics frame. */
bool lora_diag_parse(const char *message, lora_diag_t *out);

/* Parses both the legacy GPS-only frame and the attitude-extended frame.
   The attitude group counts only when complete and in range; otherwise
   has_attitude is false and attitude_rejected true, and the position (if
   any) is still returned. False only for an empty or oversized message. */
bool lora_payload_parse(const char *message, lora_payload_t *out);
