#include "ibus.h"
#include <string.h>

#define IBUS_FRAME_SIZE 32U

void IBUS_Init(ibus_state_t *state) {
    memset(state, 0, sizeof(*state));
    for (uint8_t i = 0; i < 14U; ++i) state->channel[i] = 1500U;
}

void IBUS_FeedByte(ibus_state_t *state, uint8_t byte, uint32_t now_ms) {
    static uint8_t buffer[IBUS_FRAME_SIZE];
    static uint8_t index = 0U;
    if (index == 0U && byte != 0x20U) return;
    if (index == 1U && byte != 0x40U) { index = 0U; return; }
    buffer[index++] = byte;
    if (index != IBUS_FRAME_SIZE) return;
    index = 0U;
    uint16_t checksum = 0xFFFFU;
    for (uint8_t i = 0; i < 30U; ++i) checksum -= buffer[i];
    uint16_t received = (uint16_t)buffer[30] | ((uint16_t)buffer[31] << 8U);
    if (checksum != received) return;
    for (uint8_t channel = 0; channel < 14U; ++channel) {
        uint8_t offset = 2U + 2U * channel;
        state->channel[channel] = (uint16_t)buffer[offset] | ((uint16_t)buffer[offset + 1U] << 8U);
    }
    state->last_rx_ms = now_ms;
    state->valid = true;
}
