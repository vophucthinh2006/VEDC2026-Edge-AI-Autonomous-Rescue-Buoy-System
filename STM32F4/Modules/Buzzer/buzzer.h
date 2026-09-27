#pragma once

#include <stdint.h>

/* Piezo on TIM8_CH3 (PC8), 4 kHz. Beeps are queued and played by
   Buzzer_Tick() from the main loop, so callers never block. */
void Buzzer_Init(void);
void Buzzer_Beep(uint8_t count);
void Buzzer_Tick(uint32_t now_ms);
