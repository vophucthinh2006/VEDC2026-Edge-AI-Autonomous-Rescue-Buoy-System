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
    assert(strcmp(line, "7,1,-1,0,,,0,0") == 0);

    /* An ACK with the station's position and light 1. */
    size_t n = RouteDownlink_Ack("PHAO-01", 0U, 1U, true, 10.883438, -106.796019, NULL, frame, sizeof(frame));
    assert(n > 0U && n == strlen(frame) && strncmp(frame, "ACK,PHAO-01,0,1,10.883438,-106.796019*", 38U) == 0);
    assert(ShoreLink_OnFrame(&shore, frame, "PHAO-01", 2000U, &eseq));
    assert(eseq == 0U && shore.lights == 1U && shore.lights_known);
    assert(shore.home_valid && shore.home_lat_e6 == 10883438 && shore.home_lon_e6 == -106796019);
    ShoreLink_PiLine(&shore, 8U, 4500U, line, sizeof(line));
    assert(strcmp(line, "8,1,2.5,1,10.883438,-106.796019,0,0") == 0);

    /* The station loses its fix: the home stays; an event is acknowledged; both lights. */
    RouteDownlink_Ack("PHAO-01", 42U, 3U, false, 0.0, 0.0, NULL, frame, sizeof(frame));
    assert(strcmp(frame, with_cs("ACK,PHAO-01,42,3,,")) == 0);
    assert(ShoreLink_OnFrame(&shore, frame, "PHAO-01", 3000U, &eseq));
    assert(eseq == 42U && shore.lights == 3U && shore.home_valid && shore.home_lat_e6 == 10883438);

    /* Not for this buoy, not an ACK, a bad checksum, out of range: refused, nothing changes. */
    RouteDownlink_Ack("PHAO-02", 0U, 0U, false, 0.0, 0.0, NULL, frame, sizeof(frame));
    assert(!ShoreLink_OnFrame(&shore, frame, "PHAO-01", 4000U, &eseq));
    assert(!ShoreLink_OnFrame(&shore, with_cs("RTE,PHAO-01,7,0,1,0,10.0,106.0"), "PHAO-01", 4000U, &eseq));
    assert(!ShoreLink_OnFrame(&shore, "ACK,PHAO-01,0,0,,*00", "PHAO-01", 4000U, &eseq));
    assert(!ShoreLink_OnFrame(&shore, with_cs("ACK,PHAO-01,0,16,,"), "PHAO-01", 4000U, &eseq));
    assert(!ShoreLink_OnFrame(&shore, with_cs("ACK,PHAO-01,0,0,91.0,10.0"), "PHAO-01", 4000U, &eseq));
    assert(!ShoreLink_OnFrame(&shore, with_cs("ACK,PHAO-01,0,0,10.0,"), "PHAO-01", 4000U, &eseq));
    assert(!ShoreLink_OnFrame(&shore, with_cs("ACK,PHAO-01,0,0,,,"), "PHAO-01", 4000U, &eseq));
    assert(shore.lights == 3U && shore.last_heard_ms == 3000U && shore.acks_bad == 5U);

    /* 60 s of silence loses the link; any frame of the station brings it back. */
    assert(ShoreLink_Ok(&shore, 3000U + SHORE_LINK_TIMEOUT_MS - 1U));
    assert(!ShoreLink_Ok(&shore, 3000U + SHORE_LINK_TIMEOUT_MS));
    ShoreLink_PiLine(&shore, 9U, 3000U + SHORE_LINK_TIMEOUT_MS, line, sizeof(line));
    assert(strcmp(line, "9,0,60.0,1,10.883438,-106.796019,0,0") == 0);

    /* STOP from the dashboard rides in bit 2, next to the lights; released the same way. */
    RouteDownlink_Ack("PHAO-01", 0U, 4U | 1U, false, 0.0, 0.0, NULL, frame, sizeof(frame));
    assert(ShoreLink_OnFrame(&shore, frame, "PHAO-01", 80000U, &eseq));
    assert(shore.hold && shore.lights == 1U);
    ShoreLink_PiLine(&shore, 10U, 80000U, line, sizeof(line));
    assert(line[strlen(line) - 3] == '1');
    RouteDownlink_Ack("PHAO-01", 0U, 1U, false, 0.0, 0.0, NULL, frame, sizeof(frame));
    assert(ShoreLink_OnFrame(&shore, frame, "PHAO-01", 81000U, &eseq));
    assert(!shore.hold && shore.lights == 1U);
    /* RETURN rides in bit 3 (docs/fleet-control.md), with the boat's own parking spot as the way home. */
    RouteDownlink_Ack("PHAO-02", 0U, 8U, true, 10.883500, 106.796100, NULL, frame, sizeof(frame));
    assert(!ShoreLink_OnFrame(&shore, frame, "PHAO-01", 82000U, &eseq));
    RouteDownlink_Ack("PHAO-01", 0U, 8U, true, 10.883500, 106.796100, NULL, frame, sizeof(frame));
    assert(ShoreLink_OnFrame(&shore, frame, "PHAO-01", 82000U, &eseq));
    assert(shore.ret && !shore.hold && shore.lights == 0U && shore.home_lat_e6 == 10883500);
    ShoreLink_PiLine(&shore, 11U, 82000U, line, sizeof(line));
    assert(strcmp(line, "11,1,0.0,1,10.883500,106.796100,0,1") == 0);

    /* Formation: three more fields while the boat follows a leader; the plain ACK ends it. */
    ShoreLink_FollowLine(&shore, 12U, line, sizeof(line));
    assert(strcmp(line, "12,,,") == 0);
    formation_role_t role = { .on = true, .leader = "PHAO-01", .right_m = -12, .back_m = 5 };
    size_t f = RouteDownlink_Ack("PHAO-03", 0U, 0U, true, 10.883438, 106.796019, &role, frame, sizeof(frame));
    assert(f > 0U && strncmp(frame, "ACK,PHAO-03,0,0,10.883438,106.796019,PHAO-01,-12,5*", 51U) == 0);
    assert(ShoreLink_OnFrame(&shore, frame, "PHAO-03", 83000U, &eseq));
    assert(shore.follow && strcmp(shore.leader, "PHAO-01") == 0 && shore.right_m == -12 && shore.back_m == 5 && !shore.ret);
    ShoreLink_FollowLine(&shore, 13U, line, sizeof(line));
    assert(strcmp(line, "13,PHAO-01,-12,5") == 0);
    assert(!ShoreLink_OnFrame(&shore, with_cs("ACK,PHAO-03,0,0,,,PHAO-01,-501,0"), "PHAO-03", 83000U, &eseq));
    assert(!ShoreLink_OnFrame(&shore, with_cs("ACK,PHAO-03,0,0,,,,1,0"), "PHAO-03", 83000U, &eseq));
    assert(!ShoreLink_OnFrame(&shore, with_cs("ACK,PHAO-03,0,0,,,PHAO-01,1"), "PHAO-03", 83000U, &eseq));
    assert(shore.follow);
    RouteDownlink_Ack("PHAO-03", 0U, 0U, false, 0.0, 0.0, NULL, frame, sizeof(frame));
    assert(ShoreLink_OnFrame(&shore, frame, "PHAO-03", 84000U, &eseq));
    assert(!shore.follow && shore.leader[0] == '\0');

    /* A person to leave alone (docs/victim-claim.md): eleven fields, passed to the Pi once as $SKP. */
    char skp[48];
    assert(ShoreLink_SkipLine(&shore, 3U, skp, sizeof(skp)) == 0U);
    RouteDownlink_AckSkip("PHAO-03", 5U, 0U, true, 10.883438, 106.796019, NULL, true, 10.883500, -106.796100, frame, sizeof(frame));
    assert(ShoreLink_OnFrame(&shore, frame, "PHAO-03", 85000U, &eseq) && eseq == 5U && !shore.follow);
    assert(ShoreLink_SkipLine(&shore, 4U, skp, sizeof(skp)) > 0U && strcmp(skp, "4,10.883500,-106.796100") == 0);
    assert(ShoreLink_SkipLine(&shore, 5U, skp, sizeof(skp)) == 0U);
    /* In formation as well: the role and the skip point both taken. */
    formation_role_t lead = { .on = true, .leader = "PHAO-01", .right_m = 8, .back_m = -2 };
    RouteDownlink_AckSkip("PHAO-03", 0U, 0U, true, 10.883438, 106.796019, &lead, true, 10.883600, 106.796200, frame, sizeof(frame));
    assert(ShoreLink_OnFrame(&shore, frame, "PHAO-03", 86000U, &eseq));
    assert(shore.follow && strcmp(shore.leader, "PHAO-01") == 0 && shore.right_m == 8 && shore.back_m == -2);
    assert(ShoreLink_SkipLine(&shore, 6U, skp, sizeof(skp)) > 0U && strcmp(skp, "6,10.883600,106.796200") == 0);
    /* Half a role, or a bad skip point: refused. */
    uint32_t bad = shore.acks_bad;
    assert(!ShoreLink_OnFrame(&shore, with_cs("ACK,PHAO-03,0,0,,,,5,,10.0,106.0"), "PHAO-03", 87000U, &eseq));
    assert(!ShoreLink_OnFrame(&shore, with_cs("ACK,PHAO-03,0,0,,,,,,91.0,106.0"), "PHAO-03", 87000U, &eseq));
    assert(!ShoreLink_OnFrame(&shore, with_cs("ACK,PHAO-03,0,0,,,,,,,"), "PHAO-03", 87000U, &eseq));
    assert(!ShoreLink_OnFrame(&shore, with_cs("ACK,PHAO-03,0,0,,,,,,10.0,106.0,1"), "PHAO-03", 87000U, &eseq));
    assert(shore.acks_bad == bad + 4U && shore.follow);

    ShoreLink_Heard(&shore, 70000U);          /* an RTE frame */
    assert(ShoreLink_Ok(&shore, 70001U));
    /* The tick counter wrapping round does not lose it. */
    ShoreLink_Heard(&shore, 0xFFFFFF00U);
    assert(ShoreLink_Ok(&shore, 0x00000100U));

    printf("shore_link_test: PASS\n");
    return 0;
}
