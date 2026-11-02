#include "lora_slot.h"
#include <stddef.h>
#include <string.h>

uint8_t LoraSlot_FromId(const char *id, uint8_t slots)
{
    if (slots == 0U) return 0U;
    size_t end = strlen(id), start = end;
    while (start > 0U && id[start - 1U] >= '0' && id[start - 1U] <= '9') start--;
    if (start < end && end - start <= 6U) {
        uint32_t number = 0U;
        for (size_t i = start; i < end; i++) number = number * 10U + (uint32_t)(id[i] - '0');
        return (uint8_t)((number == 0U ? 0U : number - 1U) % slots);
    }
    uint32_t hash = 0U;
    for (const char *p = id; *p != '\0'; p++) hash = hash * 31U + (uint8_t)*p;
    return (uint8_t)(hash % slots);
}

uint32_t LoraSlot_Wait(uint32_t clock_ms, uint8_t slot, uint8_t slots, uint32_t slot_ms)
{
    uint32_t frame = slot_ms * slots;
    if (frame == 0U) return 0U;
    uint32_t start = (uint32_t)(slot % slots) * slot_ms;
    uint32_t phase = clock_ms % frame;
    return phase <= start ? start - phase : frame - phase + start;
}
