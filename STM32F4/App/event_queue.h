#pragma once

/* Events the shore must hear at once, each sent in its own LoRa frame until the station
 * acknowledges it (App/shore_link.h, "ACK"):
 *
 *     ev=PHAO-01,<eseq>,V,<lat>,<lon>,<risk>,<conf>,<n>     a person (n: people reported so far)
 *     ev=PHAO-01,<eseq>,T,<lat>,<lon>,<roll>,<pitch>         overturned / heeled over, the boat's GPS
 *     ev=PHAO-01,<eseq>,C,<lat>,<lon>,<presses>              PE4: the person took the supplies
 *
 * lat and lon are empty without a position. V carries the person's position as the Pi reckons it,
 * risk 0 safe .. 3 critical and the detector's confidence 0..1. eseq counts 1..65535 (0 is "no
 * event" in an ACK). A full queue drops its oldest event; an event not acknowledged after
 * EVENT_MAX_SENDS frames is dropped too, so a station that is gone does not block newer ones.
 * Plain C, no HAL: the host tests and the simulator use it as it is. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define EVENT_QUEUE_SIZE  4U
#define EVENT_MAX_SENDS   10U

typedef enum {
    EVENT_VICTIM = 'V',
    EVENT_TILT = 'T',
    EVENT_CONFIRM = 'C'
} event_type_t;

typedef struct {
    uint16_t eseq;
    char type;                 /* event_type_t */
    bool has_pos;
    int32_t lat_e6;
    int32_t lon_e6;
    int16_t a;                 /* V: risk;          T: roll x10 */
    int16_t b;                 /* V: confidence %;  T: pitch x10 */
    uint16_t c;                /* V: people so far; C: presses */
    uint8_t sends;
} event_t;

typedef struct {
    event_t items[EVENT_QUEUE_SIZE];
    uint8_t count;             /* items[0] is the oldest, the one on air */
    uint16_t next_eseq;
    uint32_t dropped;          /* events never acknowledged (debug) */
} event_queue_t;

void EventQueue_Init(event_queue_t *queue);

/* Each returns the event's eseq. has_pos false leaves lat and lon empty in the frame. */
uint16_t EventQueue_PushVictim(event_queue_t *queue, bool has_pos, int32_t lat_e6, int32_t lon_e6,
                               uint8_t risk, uint8_t confidence_pct, uint16_t people);
uint16_t EventQueue_PushTilt(event_queue_t *queue, bool has_pos, int32_t lat_e6, int32_t lon_e6,
                             float roll_deg, float pitch_deg);
uint16_t EventQueue_PushConfirm(event_queue_t *queue, bool has_pos, int32_t lat_e6, int32_t lon_e6, uint16_t presses);

static inline bool EventQueue_Pending(const event_queue_t *queue) { return queue->count > 0U; }

/* The frame of the oldest event, counting one more send of it; 0 when the queue is empty or the
 * frame did not fit. */
size_t EventQueue_Frame(event_queue_t *queue, const char *buoy_id, char *buf, size_t size);

/* The station acknowledged eseq: that event leaves the queue. False if no such event is queued. */
bool EventQueue_Ack(event_queue_t *queue, uint16_t eseq);
