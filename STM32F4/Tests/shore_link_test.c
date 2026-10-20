/* Host test of App/shore_link.c with the station's ACK builder (esp32-lora-station/main/route_downlink.c):
 * the station's answers keep the link, bring the lights and the way home; silence loses the link. */
#include "shore_link.h"
#include "route_downlink.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static const char *with_cs(const char *body)
{
    static char out[DOWNLINK_FRAME_MAX + 1U];
    unsigned cs = 0U;
    for (const char *p = body; *p != '\0'; p++) cs ^= (unsigned char)*p;
    snprintf(out, sizeof(out), "%s*%02X", body, cs);
    return out;
}

int main(void)
{
    shore_link_t shore;
    char frame[DOWNLINK_FRAME_MAX + 1U], line[96];
    uint16_t eseq = 99U;
    ShoreLink_Init(&shore);

    /* Never heard: fine, nothing to go home to. */
    assert(ShoreLink_Ok(&shore, 1000U) && ShoreLink_Ok(&shore, 500000U));
    ShoreLink_PiLine(&shore, 7U, 1000U, line, sizeof(line));
    assert(strcmp(line, "7,1,-1,0,,,0") == 0);

    /* An ACK with the station's position and light 1. */
    size_t n = RouteDownlink_Ack("PHAO-01", 0U, 1U, true, 10.883438, -106.796019, frame, sizeof(frame));
    assert(n > 0U && n == strlen(frame) && strncmp(frame, "ACK,PHAO-01,0,1,10.883438,-106.796019*", 38U) == 0);
    assert(ShoreLink_OnFrame(&shore, frame, "PHAO-01", 2000U, &eseq));
    assert(eseq == 0U && shore.lights == 1U && shore.lights_known);
    assert(shore.home_valid && shore.home_lat_e6 == 10883438 && shore.home_lon_e6 == -106796019);
    ShoreLink_PiLine(&shore, 8U, 4500U, line, sizeof(line));
    assert(strcmp(line, "8,1,2.5,1,10.883438,-106.796019,0") == 0);

    /* The station loses its fix: the home stays; an event is acknowledged; both lights. */
    RouteDownlink_Ack("PHAO-01", 42U, 3U, false, 0.0, 0.0, frame, sizeof(frame));
    assert(strcmp(frame, with_cs("ACK,PHAO-01,42,3,,")) == 0);
    assert(ShoreLink_OnFrame(&shore, frame, "PHAO-01", 3000U, &eseq));
    assert(eseq == 42U && shore.lights == 3U && shore.home_valid && shore.home_lat_e6 == 10883438);

    /* Not for this buoy, not an ACK, a bad checksum, out of range: refused, nothing changes. */
    RouteDownlink_Ack("PHAO-02", 0U, 0U, false, 0.0, 0.0, frame, sizeof(frame));
    assert(!ShoreLink_OnFrame(&shore, frame, "PHAO-01", 4000U, &eseq));
    assert(!ShoreLink_OnFrame(&shore, with_cs("RTE,PHAO-01,7,0,1,0,10.0,106.0"), "PHAO-01", 4000U, &eseq));
    assert(!ShoreLink_OnFrame(&shore, "ACK,PHAO-01,0,0,,*00", "PHAO-01", 4000U, &eseq));
    assert(!ShoreLink_OnFrame(&shore, with_cs("ACK,PHAO-01,0,8,,"), "PHAO-01", 4000U, &eseq));
    assert(!ShoreLink_OnFrame(&shore, with_cs("ACK,PHAO-01,0,0,91.0,10.0"), "PHAO-01", 4000U, &eseq));
    assert(!ShoreLink_OnFrame(&shore, with_cs("ACK,PHAO-01,0,0,10.0,"), "PHAO-01", 4000U, &eseq));
    assert(!ShoreLink_OnFrame(&shore, with_cs("ACK,PHAO-01,0,0,,,"), "PHAO-01", 4000U, &eseq));
    assert(shore.lights == 3U && shore.last_heard_ms == 3000U && shore.acks_bad == 5U);

    /* 60 s of silence loses the link; any frame of the station brings it back. */
    assert(ShoreLink_Ok(&shore, 3000U + SHORE_LINK_TIMEOUT_MS - 1U));
    assert(!ShoreLink_Ok(&shore, 3000U + SHORE_LINK_TIMEOUT_MS));
    ShoreLink_PiLine(&shore, 9U, 3000U + SHORE_LINK_TIMEOUT_MS, line, sizeof(line));
    assert(strcmp(line, "9,0,60.0,1,10.883438,-106.796019,0") == 0);

    /* STOP from the dashboard rides in bit 2, next to the lights; released the same way. */
    RouteDownlink_Ack("PHAO-01", 0U, 4U | 1U, false, 0.0, 0.0, frame, sizeof(frame));
    assert(ShoreLink_OnFrame(&shore, frame, "PHAO-01", 80000U, &eseq));
    assert(shore.hold && shore.lights == 1U);
    ShoreLink_PiLine(&shore, 10U, 80000U, line, sizeof(line));
    assert(line[strlen(line) - 1] == '1');
    RouteDownlink_Ack("PHAO-01", 0U, 1U, false, 0.0, 0.0, frame, sizeof(frame));
    assert(ShoreLink_OnFrame(&shore, frame, "PHAO-01", 81000U, &eseq));
    assert(!shore.hold && shore.lights == 1U);
    ShoreLink_Heard(&shore, 70000U);          /* an RTE frame */
    assert(ShoreLink_Ok(&shore, 70001U));
    /* The tick counter wrapping round does not lose it. */
    ShoreLink_Heard(&shore, 0xFFFFFF00U);
    assert(ShoreLink_Ok(&shore, 0x00000100U));

    printf("shore_link_test: PASS\n");
    return 0;
}
