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
    float cam_pan_deg;            /* camera servo angle from the bow, positive right */
    uint32_t last_cam_ms;         /* 0 until the first CAM packet */
} pi_command_t;

void Protocol_Init(pi_command_t *command);
void Protocol_FeedByte(pi_command_t *command, uint8_t byte, uint32_t now_ms);
bool Protocol_Send(UART_HandleTypeDef *huart, const char *command, const char *payload);
