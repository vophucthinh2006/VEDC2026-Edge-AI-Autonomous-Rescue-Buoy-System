#include "txd_queue.h"
#include <assert.h>
#include <stdio.h>

static txd_event_t victim(uint16_t sequence, uint8_t victims) {
    txd_event_t event = { 0 };
    event.sequence = sequence;
    event.victim_report = true;
    event.has_pos = true;
    event.lat_e6 = 10762622 + sequence;
    event.lon_e6 = 106660172;
    event.risk = 2U;
    event.confidence_pct = 80U;
    event.victims = victims;
    return event;
}

int main(void) {
    txd_queue_t q;
    txd_event_t out;
    TxdQueue_Init(&q);
    assert(!TxdQueue_Pop(&q, &out));

    /* Three people confirmed in the same frames: three TXD back to back, all come out, in order.
       With the old single slot only the last one reached the shore. */
    for (uint16_t s = 1U; s <= 3U; s++) {
        txd_event_t e = victim(s, (uint8_t)s);
        assert(TxdQueue_Push(&q, &e));
    }
    for (uint16_t s = 1U; s <= 3U; s++) {
        assert(TxdQueue_Pop(&q, &out));
        assert(out.sequence == s && out.victims == s && out.lat_e6 == 10762622 + s && out.victim_report);
    }
    assert(!TxdQueue_Pop(&q, &out));

    /* Full: the newest is dropped and counted, the queued ones stay intact. */
    for (uint16_t s = 10U; s < 10U + TXD_QUEUE_SIZE; s++) {
        txd_event_t e = victim(s, 1U);
        assert(TxdQueue_Push(&q, &e));
    }
    txd_event_t extra = victim(99U, 1U);
    assert(!TxdQueue_Push(&q, &extra));
    assert(q.dropped == 1U);
    for (uint16_t s = 10U; s < 10U + TXD_QUEUE_SIZE; s++) {
        assert(TxdQueue_Pop(&q, &out) && out.sequence == s);
    }
    assert(!TxdQueue_Pop(&q, &out));

    /* The indices wrap at 256 without losing or repeating an event. */
    for (unsigned i = 0U; i < 1000U; i++) {
        txd_event_t e = victim((uint16_t)i, 0U);
        assert(TxdQueue_Push(&q, &e));
        assert(TxdQueue_Pop(&q, &out) && out.sequence == (uint16_t)i);
    }
    assert(q.dropped == 1U);

    printf("txd_queue_test: PASS\n");
    return 0;
}
