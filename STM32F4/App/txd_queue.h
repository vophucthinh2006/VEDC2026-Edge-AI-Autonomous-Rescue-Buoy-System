#pragma once

#include <stdbool.h>
#include <stdint.h>

/* $TXD events from the Pi, between the UART4 interrupt (the only producer) and the main loop (the
   only consumer). One slot was not enough: the Pi sends a TXD per person confirmed, several back to
   back when people show up in the same frames, and the main loop takes them only in its control
   tick, after up to ~25 ms of blocking telemetry; every TXD but the last was overwritten and never
   reached the shore. A full queue drops the newest (the producer must not touch the tail).
   Pure C, no HAL: host tests in Tests/. */

#define TXD_QUEUE_SIZE  8U   /* a power of two */

typedef struct {
    uint16_t sequence;            /* the Pi's TXD sequence, acknowledged with $ACK */
    bool victim_report;           /* VICTIM_FOUND or VICTIM_UPDATE: a LoRa event */
    bool has_pos;
    int32_t lat_e6;
    int32_t lon_e6;
    uint8_t confidence_pct;
    uint8_t risk;                 /* 0 safe .. 3 critical */
    uint8_t victims;              /* people counted when this TXD came in, for the beacon */
} txd_event_t;

typedef struct {
    txd_event_t items[TXD_QUEUE_SIZE];
    volatile uint8_t head;        /* written by the producer only */
    volatile uint8_t tail;        /* written by the consumer only */
    volatile uint16_t dropped;    /* TXD lost to a full queue (debug) */
} txd_queue_t;

void TxdQueue_Init(txd_queue_t *queue);
/* Producer side (interrupt). False, and dropped counted, when the queue is full. */
bool TxdQueue_Push(txd_queue_t *queue, const txd_event_t *event);
/* Consumer side (main loop). False when the queue is empty. */
bool TxdQueue_Pop(txd_queue_t *queue, txd_event_t *event);
