#pragma once

#include "main.h"
#include <stdint.h>

/* DShot300 on one timer: a circular DMA burst rewrites CCR1..CCR4 on every
   update event, so frames repeat in hardware (about 2 kHz) and callers only
   change the value. The timer must run at 300 kHz (84 MHz / 280) with its
   update DMA request set to circular, word-wide. */

#define DSHOT_STOP       0U
#define DSHOT_MIN        48U     /* lowest throttle value; 1..47 are commands */
#define DSHOT_MAX        2047U

void DShot_Init(TIM_HandleTypeDef *htim);
/* ccr_index 0..3 selects CCR1..CCR4. */
void DShot_Set(uint32_t ccr_index, uint16_t value);
