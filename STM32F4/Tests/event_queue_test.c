/* Host test of App/event_queue.c with the station's parser (esp32-lora-station/main/lora_payload.c):
 * every frame the buoy builds is one the station reads back to the same values. */
#include "event_queue.h"
#include "lora_payload.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define LORA_MAX 120U

static size_t frame_of(event_queue_t *q, char *buf, lora_event_t *ev)
{
    size_t n = EventQueue_Frame(q, "PHAO-01", buf, LORA_MAX);
    if (n > 0U) {
        assert(n == strlen(buf) && n < LORA_MAX);
        assert(lora_event_parse(buf, ev));
    }
    return n;
}

int main(void)
{
    event_queue_t q;
    char buf[LORA_MAX];
    lora_event_t ev;
    EventQueue_Init(&q);
    assert(!EventQueue_Pending(&q) && EventQueue_Frame(&q, "PHAO-01", buf, sizeof(buf)) == 0U);

    /* A person, overturned, the button: in order, each until acknowledged. */
    uint16_t v = EventQueue_PushVictim(&q, true, 10883500, -106796100, 3U, 87U, 2U);
    uint16_t t = EventQueue_PushTilt(&q, false, 0, 0, -120.46f, 4.04f);
    uint16_t c = EventQueue_PushConfirm(&q, true, 10883438, 106796019, 4U);
    assert(v == 1U && t == 2U && c == 3U && q.count == 3U);

    assert(frame_of(&q, buf, &ev) > 0U);
    assert(strcmp(buf, "ev=PHAO-01,1,V,10.883500,-106.796100,3,0.87,2") == 0);
    assert(ev.type == 'V' && ev.seq == 1U && ev.risk == 3U && fabsf(ev.confidence - 0.87f) < 1e-4f && ev.count == 2U);
    assert(frame_of(&q, buf, &ev) > 0U && ev.seq == 1U);   /* no ACK yet: the same one again */
    assert(!EventQueue_Ack(&q, 0U) && !EventQueue_Ack(&q, 77U));
    assert(EventQueue_Ack(&q, v) && q.count == 2U);

    assert(frame_of(&q, buf, &ev) > 0U);
    assert(strcmp(buf, "ev=PHAO-01,2,T,,,-120.5,4.0") == 0 && !ev.has_position);
    assert(EventQueue_Ack(&q, t));
    assert(frame_of(&q, buf, &ev) > 0U);
    assert(strcmp(buf, "ev=PHAO-01,3,C,10.883438,106.796019,4") == 0 && ev.presses == 4U);
    assert(EventQueue_Ack(&q, c) && !EventQueue_Pending(&q));

    /* Confidence edges and a person with no position. */
    EventQueue_PushVictim(&q, false, 0, 0, 0U, 5U, 1U);
    assert(frame_of(&q, buf, &ev) > 0U && strcmp(buf, "ev=PHAO-01,4,V,,,0,0.05,1") == 0);
    EventQueue_Ack(&q, 4U);
    EventQueue_PushVictim(&q, true, 1, -1, 9U, 200U, 255U);  /* clamped to critical, 100 % */
    assert(frame_of(&q, buf, &ev) > 0U && strcmp(buf, "ev=PHAO-01,5,V,0.000001,-0.000001,3,1.00,255") == 0);
    EventQueue_Ack(&q, 5U);

    /* Never acknowledged: dropped after EVENT_MAX_SENDS frames, the next one goes. */
    EventQueue_PushConfirm(&q, false, 0, 0, 1U);
    EventQueue_PushConfirm(&q, false, 0, 0, 2U);
    for (unsigned i = 0U; i < EVENT_MAX_SENDS; i++) { assert(frame_of(&q, buf, &ev) > 0U); assert(ev.presses == 1U); }
    assert(frame_of(&q, buf, &ev) > 0U && ev.presses == 2U && q.dropped == 1U);
    EventQueue_Ack(&q, ev.seq);

    /* A full queue drops its oldest. */
    for (uint16_t i = 1U; i <= EVENT_QUEUE_SIZE + 1U; i++) EventQueue_PushConfirm(&q, false, 0, 0, i);
    assert(q.count == EVENT_QUEUE_SIZE && q.dropped == 2U);
    assert(frame_of(&q, buf, &ev) > 0U && ev.presses == 2U);

    /* eseq wraps past 65535 to 1, never 0. */
    q.next_eseq = 0xFFFFU;
    assert(EventQueue_PushConfirm(&q, false, 0, 0, 1U) == 0xFFFFU);
    assert(EventQueue_PushConfirm(&q, false, 0, 0, 1U) == 1U);

    /* The longest frames still fit the radio. */
    EventQueue_Init(&q);
    EventQueue_PushTilt(&q, true, -89999999, -179999999, -179.9f, -89.9f);
    q.items[0].eseq = 65535U;
    assert(frame_of(&q, buf, &ev) > 0U && ev.has_position);

    printf("event_queue_test: PASS\n");
    return 0;
}
