#include "txd_queue.h"
#include <stdatomic.h>
#include <string.h>

#define MASK (TXD_QUEUE_SIZE - 1U)

_Static_assert((TXD_QUEUE_SIZE & MASK) == 0U, "TXD_QUEUE_SIZE must be a power of two");

void TxdQueue_Init(txd_queue_t *queue) { memset((void *)queue, 0, sizeof(*queue)); }

bool TxdQueue_Push(txd_queue_t *queue, const txd_event_t *event) {
    uint8_t head = queue->head;
    if ((uint8_t)(head - queue->tail) >= TXD_QUEUE_SIZE) {
        queue->dropped++;
        return false;
    }
    queue->items[head & MASK] = *event;
    /* The item is written before the consumer can see the new head. */
    atomic_signal_fence(memory_order_release);
    queue->head = (uint8_t)(head + 1U);
    return true;
}

bool TxdQueue_Pop(txd_queue_t *queue, txd_event_t *event) {
    uint8_t tail = queue->tail;
    if (tail == queue->head) return false;
    atomic_signal_fence(memory_order_acquire);
    *event = queue->items[tail & MASK];
    /* The item is read before the producer may reuse its slot. */
    atomic_signal_fence(memory_order_release);
    queue->tail = (uint8_t)(tail + 1U);
    return true;
}
