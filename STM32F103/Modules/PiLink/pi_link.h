#pragma once

/* Line protocol with the Raspberry Pi on USART1, the same framing as the STM32F4 boat
 * ($BODY*CS\n, CS = XOR of the body in hex; Rasp_Pi/utils/nmea_packet.py). No HAL in here: bytes
 * in, state out, so it is tested on the PC (Tests/helper_test.c).
 *
 * Pi -> board:
 *   $HBT,seq,state                 the Pi is alive
 *   $CAM,seq,pan_deg               camera servo angle from the bow, positive right
 *   $VIC,seq,count                 people found so far (absolute, repeated every second)
 *   $POS,seq,fix,lat,lon,roll,pitch,yaw,target,mode,imu_ok,calib
 *                                  what the LoRa frame carries; the numbers are kept as the text
 *                                  the Pi wrote, the board does no floating point on them
 * Board -> Pi:
 *   $AUX,seq,battery_mv,current_ma,estop,lora_ok,alarm
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PI_LINK_MAX_LINE   128U
#define PI_LINK_STATUS_MAX 72U

typedef struct {
    bool seen;                 /* any valid HBT since boot */
    uint32_t last_hbt_ms;
    bool cam_seen;
    int16_t cam_pan_ddeg;      /* tenths of a degree, positive right */
    uint32_t last_cam_ms;
    uint8_t victims;
    bool status_seen;
    uint32_t last_status_ms;
    /* Middle of the LoRa frame, ready to print: "lat,lon,r=..,p=..,y=..,t=..,m=A,i=1,c=3" or
       "NO_FIX,r=..". */
    char status[PI_LINK_STATUS_MAX];
    uint16_t rx_ok, rx_bad;    /* lines accepted / dropped (checksum, format) */
    char line[PI_LINK_MAX_LINE];
    uint8_t length;
} pi_link_t;

void PiLink_Init(pi_link_t *link);
void PiLink_FeedByte(pi_link_t *link, uint8_t byte, uint32_t now_ms);
/* Writes "$command,payload*CS\n" to out. Returns its length, 0 if it does not fit. */
size_t PiLink_Encode(char *out, size_t size, const char *command, const char *payload);
