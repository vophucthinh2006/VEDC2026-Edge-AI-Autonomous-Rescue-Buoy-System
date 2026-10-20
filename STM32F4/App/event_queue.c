#include "event_queue.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

void EventQueue_Init(event_queue_t *queue)
{
    memset(queue, 0, sizeof(*queue));
    queue->next_eseq = 1U;
}

static void remove_at(event_queue_t *queue, uint8_t index)
{
    for (uint8_t i = index; i + 1U < queue->count; i++) queue->items[i] = queue->items[i + 1U];
    queue->count--;
}

static event_t *push(event_queue_t *queue, char type, bool has_pos, int32_t lat_e6, int32_t lon_e6)
{
    if (queue->count == EVENT_QUEUE_SIZE) {
        remove_at(queue, 0U);
        queue->dropped++;
    }
    event_t *event = &queue->items[queue->count++];
    memset(event, 0, sizeof(*event));
    event->eseq = queue->next_eseq;
    queue->next_eseq = (uint16_t)(queue->next_eseq + 1U);
    if (queue->next_eseq == 0U) queue->next_eseq = 1U;
    event->type = type;
    event->has_pos = has_pos;
    event->lat_e6 = has_pos ? lat_e6 : 0;
    event->lon_e6 = has_pos ? lon_e6 : 0;
    return event;
}

static int16_t tenths(float deg)
{
    float clamped = deg > 3000.0f ? 3000.0f : (deg < -3000.0f ? -3000.0f : deg);
    return (int16_t)lroundf(clamped * 10.0f);
}

uint16_t EventQueue_PushVictim(event_queue_t *queue, bool has_pos, int32_t lat_e6, int32_t lon_e6,
                               uint8_t risk, uint8_t confidence_pct, uint16_t people)
{
    event_t *event = push(queue, EVENT_VICTIM, has_pos, lat_e6, lon_e6);
    event->a = (int16_t)(risk > 3U ? 3U : risk);
    event->b = (int16_t)(confidence_pct > 100U ? 100U : confidence_pct);
    event->c = people;
    return event->eseq;
}

uint16_t EventQueue_PushTilt(event_queue_t *queue, bool has_pos, int32_t lat_e6, int32_t lon_e6,
                             float roll_deg, float pitch_deg)
{
    event_t *event = push(queue, EVENT_TILT, has_pos, lat_e6, lon_e6);
    event->a = tenths(roll_deg);
    event->b = tenths(pitch_deg);
    return event->eseq;
}

uint16_t EventQueue_PushConfirm(event_queue_t *queue, bool has_pos, int32_t lat_e6, int32_t lon_e6, uint16_t presses)
{
    event_t *event = push(queue, EVENT_CONFIRM, has_pos, lat_e6, lon_e6);
    event->c = presses;
    return event->eseq;
}

/* "10.883438" from 10883438, "" without a position. */
static void format_e6(char *buf, size_t size, bool has, int32_t value)
{
    if (!has) { buf[0] = '\0'; return; }
    uint32_t magnitude = value < 0 ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
    snprintf(buf, size, "%s%lu.%06lu", value < 0 ? "-" : "", (unsigned long)(magnitude / 1000000UL),
             (unsigned long)(magnitude % 1000000UL));
}

/* "-12.3" from -123. */
static void format_tenths(char *buf, size_t size, int16_t value)
{
    unsigned magnitude = (unsigned)(value < 0 ? -(int)value : (int)value);
    snprintf(buf, size, "%s%u.%u", value < 0 ? "-" : "", magnitude / 10U, magnitude % 10U);
}

size_t EventQueue_Frame(event_queue_t *queue, const char *buoy_id, char *buf, size_t size)
{
    /* An event the station never acknowledged gives way to the next one. */
    while (queue->count > 0U && queue->items[0].sends >= EVENT_MAX_SENDS) {
        remove_at(queue, 0U);
        queue->dropped++;
    }
    if (queue->count == 0U || size == 0U) return 0U;
    event_t *event = &queue->items[0];
    char lat[16], lon[16], a[12], b[12];
    format_e6(lat, sizeof(lat), event->has_pos, event->lat_e6);
    format_e6(lon, sizeof(lon), event->has_pos, event->lon_e6);
    int n;
    switch (event->type) {
    case EVENT_VICTIM:
        n = snprintf(buf, size, "ev=%s,%u,V,%s,%s,%d,%d.%02d,%u", buoy_id, (unsigned)event->eseq, lat, lon,
                     (int)event->a, (int)event->b / 100, (int)event->b % 100, (unsigned)event->c);
        break;
    case EVENT_TILT:
        format_tenths(a, sizeof(a), event->a);
        format_tenths(b, sizeof(b), event->b);
        n = snprintf(buf, size, "ev=%s,%u,T,%s,%s,%s,%s", buoy_id, (unsigned)event->eseq, lat, lon, a, b);
        break;
    default:
        n = snprintf(buf, size, "ev=%s,%u,C,%s,%s,%u", buoy_id, (unsigned)event->eseq, lat, lon, (unsigned)event->c);
        break;
    }
    if (n < 0 || (size_t)n >= size) { buf[0] = '\0'; return 0U; }
    event->sends++;
    return (size_t)n;
}

bool EventQueue_Ack(event_queue_t *queue, uint16_t eseq)
{
    if (eseq == 0U) return false;
    for (uint8_t i = 0U; i < queue->count; i++) {
        if (queue->items[i].eseq == eseq) {
            remove_at(queue, i);
            return true;
        }
    }
    return false;
}
