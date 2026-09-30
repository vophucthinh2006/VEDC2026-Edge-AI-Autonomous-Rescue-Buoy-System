#include "dshot.h"
#include <string.h>

#define DSHOT_BITS        16U
#define DSHOT_FRAME_SLOTS 150U   /* 16 bit slots + 134 low slots: ~2 kHz frame rate */
#define DSHOT_BIT1        210U   /* 75 % of the 280-count bit */
#define DSHOT_BIT0        105U   /* 37.5 % */

/* One row per bit slot, one column per CCR. Rows past the 16th stay 0 and
   hold the line low between frames. */
static uint32_t burst[DSHOT_FRAME_SLOTS][4];

static uint16_t frame(uint16_t value) {
    uint16_t packet = (uint16_t)(value << 1);   /* telemetry request off */
    uint16_t crc = (uint16_t)((packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0FU);
    return (uint16_t)((packet << 4) | crc);
}

void DShot_Set(uint32_t ccr_index, uint16_t value) {
    if (ccr_index > 3U) return;
    if (value > DSHOT_MAX) value = DSHOT_MAX;
    /* The DMA may read mid-update; a torn frame fails its CRC and the ESC
       simply keeps the previous value until the next clean frame. */
    uint16_t bits = frame(value);
    for (uint32_t b = 0; b < DSHOT_BITS; b++)
        burst[b][ccr_index] = (bits & (0x8000U >> b)) ? DSHOT_BIT1 : DSHOT_BIT0;
}

void DShot_Init(TIM_HandleTypeDef *htim) {
    memset(burst, 0, sizeof(burst));
    HAL_TIM_DMABurst_MultiWriteStart(htim, TIM_DMABASE_CCR1, TIM_DMA_UPDATE, (uint32_t *)burst,
                                     TIM_DMABURSTLENGTH_4TRANSFERS, DSHOT_FRAME_SLOTS * 4U);
}
