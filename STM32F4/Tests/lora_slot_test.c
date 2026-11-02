/* Host test of App/lora_slot.c: each boat gets its own slot, and the wait lands on its start. */
#include "lora_slot.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    assert(LoraSlot_FromId("PHAO-01", LORA_SLOT_COUNT) == 0U);
    assert(LoraSlot_FromId("PHAO-02", LORA_SLOT_COUNT) == 1U);
    assert(LoraSlot_FromId("PHAO-04", LORA_SLOT_COUNT) == 3U);
    assert(LoraSlot_FromId("PHAO-05", LORA_SLOT_COUNT) == 0U);   /* a fifth boat shares PHAO-01's slot */
    assert(LoraSlot_FromId("SIM-00", LORA_SLOT_COUNT) == 0U);
    assert(LoraSlot_FromId("ALPHA", LORA_SLOT_COUNT) == LoraSlot_FromId("ALPHA", LORA_SLOT_COUNT));
    assert(LoraSlot_FromId("ALPHA", LORA_SLOT_COUNT) < LORA_SLOT_COUNT);
    assert(LoraSlot_FromId("PHAO-02", 0U) == 0U);

    /* Slot 1 starts 2 s into each 8 s frame. */
    assert(LoraSlot_Wait(0U, 1U, 4U, 2000U) == 2000U);
    assert(LoraSlot_Wait(2000U, 1U, 4U, 2000U) == 0U);
    assert(LoraSlot_Wait(2001U, 1U, 4U, 2000U) == 7999U);
    assert(LoraSlot_Wait(16500U, 1U, 4U, 2000U) == 1500U);
    assert(LoraSlot_Wait(7999U, 0U, 4U, 2000U) == 1U);
    /* Every boat's start falls on its own slot, never on another's. */
    for (uint32_t t = 0U; t < 3U * LORA_SLOT_FRAME_MS; t += 137U) {
        for (uint8_t s = 0U; s < LORA_SLOT_COUNT; s++) {
            uint32_t at = t + LoraSlot_Wait(t, s, LORA_SLOT_COUNT, LORA_SLOT_MS);
            assert(at % LORA_SLOT_FRAME_MS == (uint32_t)s * LORA_SLOT_MS);
            assert(at - t < LORA_SLOT_FRAME_MS);
        }
    }
    /* Near the 32-bit wrap of a millisecond clock. */
    assert(LoraSlot_Wait(86399999U, 0U, 4U, 2000U) < LORA_SLOT_FRAME_MS);

    printf("lora_slot_test: PASS\n");
    return 0;
}
