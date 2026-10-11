#include "ibus_sensor.h"
#include <string.h>

#define CMD_DISCOVER 0x80U
#define CMD_TYPE     0x90U
#define CMD_VALUE    0xA0U

static void put_checksum(uint8_t *frame, uint8_t length) {
    uint16_t checksum = 0xFFFFU;
    for (uint8_t i = 0U; i < length; i++) checksum -= frame[i];
    frame[length] = (uint8_t)checksum;
    frame[length + 1U] = (uint8_t)(checksum >> 8);
}

void IbusSensor_Init(ibus_sensor_t *sensor) { memset(sensor, 0, sizeof(*sensor)); }

uint8_t IbusSensor_FeedByte(ibus_sensor_t *sensor, uint8_t byte, uint8_t reply[IBUS_SENSOR_REPLY_MAX]) {
    /* Every poll starts with its length, 4. Anything else on the wire (other sensors' answers
       start with 6) is skipped until the next 4. */
    if (sensor->index == 0U && byte != 0x04U) return 0U;
    sensor->frame[sensor->index++] = byte;
    if (sensor->index < 4U) return 0U;
    sensor->index = 0U;

    uint8_t expected[4] = { sensor->frame[0], sensor->frame[1], 0U, 0U };
    put_checksum(expected, 2U);
    if (expected[2] != sensor->frame[2] || expected[3] != sensor->frame[3]) return 0U;
    if ((sensor->frame[1] & 0x0FU) != IBUS_SENSOR_ADDRESS) return 0U;
    sensor->polls++;

    switch (sensor->frame[1] & 0xF0U) {
    case CMD_DISCOVER:
        memcpy(reply, sensor->frame, 4U);
        return 4U;
    case CMD_TYPE:
        reply[0] = 0x06U; reply[1] = sensor->frame[1]; reply[2] = IBUS_SENSOR_TYPE_EXT_VOLT; reply[3] = 0x02U;   /* 2-byte value */
        put_checksum(reply, 4U);
        return 6U;
    case CMD_VALUE:
        reply[0] = 0x06U; reply[1] = sensor->frame[1]; reply[2] = (uint8_t)sensor->value; reply[3] = (uint8_t)(sensor->value >> 8);
        put_checksum(reply, 4U);
        return 6U;
    default:
        return 0U;
    }
}
