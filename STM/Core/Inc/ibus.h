#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t channel[14];
    uint32_t last_rx_ms;
    bool valid;
} ibus_state_t;

void IBUS_Init(ibus_state_t *state);
void IBUS_FeedByte(ibus_state_t *state, uint8_t byte, uint32_t now_ms);
