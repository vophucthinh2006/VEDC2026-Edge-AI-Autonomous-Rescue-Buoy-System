#pragma once

/* One telemetry sensor on the FS-iA6B's "i-BUS SENS" port: the value shows up on the FS-i6
 * handset. The receiver is the master of a single half-duplex wire at 115200 baud and polls
 * every sensor address in turn with 4-byte frames; the sensor answers each within a few ms.
 *
 *   04 8a ck ck          anybody at address a?        answer: the same four bytes
 *   04 9a ck ck          what are you?                answer: 06 9a type 02 ck ck
 *   04 Aa ck ck          your value?                  answer: 06 Aa lo hi ck ck
 *
 * ck = 0xFFFF minus the sum of the bytes before it, low byte first. No HAL in here: bytes in,
 * the answer out (tested on the PC, Tests/helper_test.c). */

#include <stdint.h>

#define IBUS_SENSOR_ADDRESS        1U      /* address 0 is the receiver's own voltage */
#define IBUS_SENSOR_TYPE_EXT_VOLT  0x03U   /* external voltage, 0.01 V per count */
#define IBUS_SENSOR_REPLY_MAX      6U

typedef struct {
    uint8_t frame[4];
    uint8_t index;
    uint16_t value;      /* what the next "your value?" is answered with */
    uint16_t polls;      /* frames addressed to this sensor */
} ibus_sensor_t;

void IbusSensor_Init(ibus_sensor_t *sensor);
/* Returns the length of the answer written to reply, 0 when this byte ends no frame for us. */
uint8_t IbusSensor_FeedByte(ibus_sensor_t *sensor, uint8_t byte, uint8_t reply[IBUS_SENSOR_REPLY_MAX]);
