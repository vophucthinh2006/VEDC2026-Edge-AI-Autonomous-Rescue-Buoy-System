#pragma once

/* The beacon line the shore station parses (esp32-lora-station/main/lora_payload.c), the same as
 * the STM32F4 boat sends:
 *   id=PHAO-01,10.762622,106.660172,r=2.1,p=-1.4,y=278.5,t=280.0,m=A,i=1,c=3,q=12,v=0
 *   id=PHAO-01,NO_FIX,r=2.1,p=-1.4,y=278.5,t=-1.0,m=S,i=0,c=0,q=13,v=1
 * No HAL in here (tested on the PC, Tests/helper_test.c). */

#include <stddef.h>
#include <stdint.h>

#define LORA_FRAME_SIZE 100U   /* the station keeps 100 bytes */

/* status: the middle of the line as the Pi link holds it, or NULL when the Pi has said nothing
   recent: the beacon then goes out as NO_FIX with no attitude, so the shore still sees the board
   and its victim count. Returns the length, 0 if the line does not fit. */
size_t LoraFrame_Build(char *out, size_t size, const char *id, const char *status, uint32_t sequence, uint8_t victims);
