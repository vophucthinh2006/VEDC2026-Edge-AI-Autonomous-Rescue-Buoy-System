/* PC tests of the parts with no HAL in them: Tools\run_host_tests.ps1 */
#include "ibus_sensor.h"
#include "lora_frame.h"
#include "pi_link.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void feed(pi_link_t *link, const char *command, const char *payload, uint32_t now_ms) {
    char line[PI_LINK_MAX_LINE];
    size_t length = PiLink_Encode(line, sizeof(line), command, payload);
    assert(length > 0U);
    for (size_t i = 0U; i < length; i++) PiLink_FeedByte(link, (uint8_t)line[i], now_ms);
}

static void feed_raw(pi_link_t *link, const char *text) {
    for (; *text; text++) PiLink_FeedByte(link, (uint8_t)*text, 0U);
}

static void test_pi_link(void) {
    pi_link_t link;
    char line[PI_LINK_MAX_LINE];
    PiLink_Init(&link);

    /* Same framing as Rasp_Pi/utils/nmea_packet.py: XOR of the body, two hex digits. */
    assert(PiLink_Encode(line, sizeof(line), "AUX", "1,12000,350,0,1,0") > 0U);
    assert(strcmp(line, "$AUX,1,12000,350,0,1,0*49\n") == 0);     /* what nmea_packet.encode gives */
    assert(PiLink_Encode(line, 8U, "AUX", "1,12000,350,0,1,0") == 0U);

    feed(&link, "HBT", "7,search", 100U);
    assert(link.seen && link.last_hbt_ms == 100U && link.rx_ok == 1U);

    feed(&link, "CAM", "8,-12.5", 200U);
    assert(link.cam_seen && link.cam_pan_ddeg == -125 && link.last_cam_ms == 200U);
    feed(&link, "CAM", "9,60", 210U);
    assert(link.cam_pan_ddeg == 600);
    feed(&link, "CAM", "10,95.0", 220U);              /* beyond the servo: kept out */
    feed(&link, "CAM", "11,1e2", 230U);
    assert(link.cam_pan_ddeg == 600 && link.rx_bad == 2U);

    feed(&link, "VIC", "12,3", 300U);
    assert(link.victims == 3U);
    feed(&link, "VIC", "13,256", 310U);
    feed(&link, "VIC", "14,-1", 320U);
    assert(link.victims == 3U && link.rx_bad == 4U);

    feed(&link, "POS", "15,1,10.762622,106.660172,2.1,-1.4,278.5,-1.0,A,1,3", 400U);
    assert(link.status_seen && link.last_status_ms == 400U);
    assert(strcmp(link.status, "10.762622,106.660172,r=2.1,p=-1.4,y=278.5,t=-1.0,m=A,i=1,c=3") == 0);
    feed(&link, "POS", "16,0,0.000000,0.000000,0.0,0.0,90.0,-1.0,S,1,0", 500U);
    assert(strcmp(link.status, "NO_FIX,r=0.0,p=0.0,y=90.0,t=-1.0,m=S,i=1,c=0") == 0);
    /* A field that is not what it should be leaves the last good status alone. */
    feed(&link, "POS", "17,1,10.7,106.6,2.1,-1.4,278.5,-1.0,X,1,3", 600U);
    feed(&link, "POS", "18,1,10.7,106.6,2.1,q=9,278.5,-1.0,A,1,3", 610U);
    feed(&link, "POS", "19,1,10.7,106.6", 620U);
    assert(strcmp(link.status, "NO_FIX,r=0.0,p=0.0,y=90.0,t=-1.0,m=S,i=1,c=0") == 0 && link.last_status_ms == 500U);

    uint16_t bad = link.rx_bad;
    feed_raw(&link, "$HBT,1,x*00\n");                 /* wrong checksum */
    feed_raw(&link, "garbage\n");
    assert(link.rx_bad == bad + 2U);
    feed_raw(&link, "$HBT,1,x*2");                    /* a line cut short runs into the next one: both lost */
    feed(&link, "HBT", "20,search", 700U);
    assert(link.last_hbt_ms == 100U && link.rx_bad == bad + 3U);
    feed(&link, "HBT", "21,search", 800U);            /* and the one after is read again */
    assert(link.last_hbt_ms == 800U);
}

static void test_lora_frame(void) {
    char frame[LORA_FRAME_SIZE];
    size_t n = LoraFrame_Build(frame, sizeof(frame), "PHAO-01", "10.762622,106.660172,r=2.1,p=-1.4,y=278.5,t=280.0,m=A,i=1,c=3", 12U, 0U);
    assert(n == strlen(frame));
    assert(strcmp(frame, "id=PHAO-01,10.762622,106.660172,r=2.1,p=-1.4,y=278.5,t=280.0,m=A,i=1,c=3,q=12,v=0") == 0);
    LoraFrame_Build(frame, sizeof(frame), "PHAO-01", NULL, 13U, 1U);
    assert(strcmp(frame, "id=PHAO-01,NO_FIX,r=0.0,p=0.0,y=0.0,t=-1.0,m=S,i=0,c=0,q=13,v=1") == 0);
    /* The longest line the Pi link lets through still fits the station's 100 bytes. */
    n = LoraFrame_Build(frame, sizeof(frame), "PHAO-01", "-10.762622,-106.660172,r=-179.9,p=-89.9,y=359.9,t=359.9,m=A,i=1,c=3", 4294967295UL, 255U);
    assert(n > 0U && n < 100U);
    assert(LoraFrame_Build(frame, 40U, "PHAO-01", NULL, 1U, 0U) == 0U);
}

static uint8_t poll(ibus_sensor_t *sensor, uint8_t command, uint8_t reply[IBUS_SENSOR_REPLY_MAX]) {
    uint16_t checksum = (uint16_t)(0xFFFFU - 0x04U - command);
    uint8_t frame[4] = { 0x04U, command, (uint8_t)checksum, (uint8_t)(checksum >> 8) };
    uint8_t length = 0U;
    for (int i = 0; i < 4; i++) length = IbusSensor_FeedByte(sensor, frame[i], reply);
    return length;
}

static void test_ibus_sensor(void) {
    ibus_sensor_t sensor;
    uint8_t reply[IBUS_SENSOR_REPLY_MAX];
    IbusSensor_Init(&sensor);
    sensor.value = 1234U;                             /* 12.34 V */

    assert(poll(&sensor, 0x81U, reply) == 4U);        /* discovery: echoed */
    assert(reply[0] == 0x04U && reply[1] == 0x81U && reply[2] == 0x7AU && reply[3] == 0xFFU);

    assert(poll(&sensor, 0x91U, reply) == 6U);        /* type: external voltage, two bytes */
    assert(reply[0] == 0x06U && reply[1] == 0x91U && reply[2] == 0x03U && reply[3] == 0x02U);
    assert((uint16_t)(reply[4] | reply[5] << 8) == (uint16_t)(0xFFFFU - 0x06U - 0x91U - 0x03U - 0x02U));

    assert(poll(&sensor, 0xA1U, reply) == 6U);        /* value, low byte first */
    assert(reply[2] == 0xD2U && reply[3] == 0x04U);
    assert((uint16_t)(reply[4] | reply[5] << 8) == (uint16_t)(0xFFFFU - 0x06U - 0xA1U - 0xD2U - 0x04U));

    assert(poll(&sensor, 0xA2U, reply) == 0U);        /* another sensor's address */
    assert(sensor.polls == 3U);
    /* Another sensor's answer on the wire, then a poll with a bad checksum: both ignored. */
    const uint8_t noise[] = { 0x06U, 0xA2U, 0x10U, 0x00U, 0x47U, 0xFFU, 0x04U, 0xA1U, 0x00U, 0x00U };
    for (size_t i = 0U; i < sizeof(noise); i++) assert(IbusSensor_FeedByte(&sensor, noise[i], reply) == 0U);
    assert(poll(&sensor, 0xA1U, reply) == 6U);
}

int main(void) {
    test_pi_link();
    test_lora_frame();
    test_ibus_sensor();
    puts("helper_test: PASS");
    return 0;
}
