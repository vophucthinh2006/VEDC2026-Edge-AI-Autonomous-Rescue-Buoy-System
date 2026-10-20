#pragma once

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum { NAV_STOP, NAV_AUTO, NAV_MANUAL } nav_mode_t;
typedef struct {
    nav_mode_t mode;
    float speed_mps;
    float heading_deg;
    uint16_t ttl_ms;
    uint16_t sequence;
    uint32_t last_nav_ms;
    uint32_t last_hbt_ms;
    bool pi_link_ok;
    bool sos_on;
    bool txd_pending;             /* Event accepted from Pi, to be acknowledged by the main loop. */
    uint16_t txd_sequence;
    uint8_t victim_count;         /* VICTIM_FOUND events since boot, stops at 255; sent to shore in the LoRa beacon */
    /* The last VICTIM_FOUND (a new person) or VICTIM_UPDATE (the one followed: new risk or position):
       $TXD,seq,VICTIM_FOUND|VICTIM_UPDATE,lat,lon,confidence[,risk]. lat = lon = 0 is "no
       position" (the Pi had no GPS fix). risk 0 safe .. 3 critical; an older Pi sends none: 2 (high). */
    bool victim_report;           /* this TXD was a VICTIM_FOUND, for the LoRa event */
    bool victim_has_pos;
    int32_t victim_lat_e6;
    int32_t victim_lon_e6;
    uint8_t victim_confidence_pct;
    uint8_t victim_risk;
    float cam_pan_deg;            /* camera servo angle from the bow, positive right */
    uint32_t last_cam_ms;         /* 0 until the first CAM packet */
    uint16_t route_rev;           /* LoRa route the Pi reports in $HBT, 0 = none (App/route_link.h) */
} pi_command_t;

void Protocol_Init(pi_command_t *command);
void Protocol_FeedByte(pi_command_t *command, uint8_t byte, uint32_t now_ms);
bool Protocol_Send(UART_HandleTypeDef *huart, const char *command, const char *payload);
