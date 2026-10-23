#pragma once

#include "main.h"
#include "txd_queue.h"
#include <stdbool.h>
#include <stdint.h>

/* NAV_BACK: $NAV,seq,BACK,speed,heading,ttl, the front motor alone pulls the boat astern at speed as a
   throttle (App/control.c), off an obstacle it is wedged against. An older F407 takes it for STOP. */
typedef enum { NAV_STOP, NAV_AUTO, NAV_MANUAL, NAV_BACK } nav_mode_t;
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
    uint8_t victim_count;         /* VICTIM_FOUND events since boot, stops at 255; sent to shore in the LoRa beacon */
    /* Events accepted from the Pi, each acknowledged by the main loop and, for a VICTIM_FOUND (a new
       person) or VICTIM_UPDATE (the one followed: new risk or position), handed to the LoRa beacon:
       $TXD,seq,VICTIM_FOUND|VICTIM_UPDATE,lat,lon,confidence[,risk]. lat = lon = 0 is "no
       position" (the Pi had no GPS fix). risk 0 safe .. 3 critical; an older Pi sends none: 2 (high). */
    txd_queue_t txd;
    float cam_pan_deg;            /* camera servo angle from the bow, positive right */
    uint32_t last_cam_ms;         /* 0 until the first CAM packet */
    uint16_t route_rev;           /* LoRa route the Pi reports in $HBT, 0 = none (App/route_link.h) */
    /* $HBT,seq,reason,route_rev[,mission,wp/total[,fps_x10]]: the reason of the Pi's NAV command, its
       mission (I P F R B H), the waypoint it goes to and its detector's frames a second x10 ("-" while
       the detector is off), for the dashboard (App/diag_frame.h). */
    char pi_reason[24];
    char pi_mission;              /* 0 until a Pi sends it */
    uint16_t pi_wp;
    uint16_t pi_wp_total;
    int16_t pi_fps_x10;           /* -1 unknown or the detector off */
} pi_command_t;

void Protocol_Init(pi_command_t *command);
void Protocol_FeedByte(pi_command_t *command, uint8_t byte, uint32_t now_ms);
bool Protocol_Send(UART_HandleTypeDef *huart, const char *command, const char *payload);
