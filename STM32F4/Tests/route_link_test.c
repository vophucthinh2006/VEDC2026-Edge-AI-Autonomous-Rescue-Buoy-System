/* Host test of App/route_link.c with the station's frame builder (esp32-lora-station/main/route_downlink.c):
 * a whole route goes across, frame by frame, the way the two radios pass it. */
#include "route_link.h"
#include "route_downlink.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static route_link_t buoy;
static route_downlink_t station;

/* A hand-written frame with its "*CS". */
static const char *with_cs(const char *body)
{
    static char out[DOWNLINK_FRAME_MAX + 1U];
    unsigned cs = 0U;
    for (const char *p = body; *p != '\0'; p++) cs ^= (unsigned char)*p;
    snprintf(out, sizeof(out), "%s*%02X", body, cs);
    return out;
}

/* One beacon / answer round: the buoy's w= goes to the station, its frame comes back. */
static size_t exchange(char *frame)
{
    char field[32];
    route_ack_t ack = { 0 };
    RouteLink_BeaconField(&buoy, field, sizeof(field));
    if (field[0] != '\0') assert(RouteDownlink_ParseAck(field + 2, &ack));
    size_t n = RouteDownlink_Frame(&station, "PHAO-01", &ack, frame, DOWNLINK_FRAME_MAX + 1U);
    if (n > 0U) {
        assert(n <= DOWNLINK_FRAME_MAX);
        assert(RouteLink_OnFrame(&buoy, frame, "PHAO-01"));
    }
    return n;
}

int main(void)
{
    char frame[DOWNLINK_FRAME_MAX + 1U], field[32], line[192];
    RouteLink_Init(&buoy);
    RouteLink_BeaconField(&buoy, field, sizeof(field));
    assert(field[0] == '\0');                         /* no route yet: the beacon is as before */

    /* A 40-point route, negative coordinates included. */
    memset(&station, 0, sizeof(station));
    station.rev = 7U;
    strcpy(station.id, "PHAO-01");
    station.count = 40U;
    for (unsigned i = 0U; i < station.count; i++) {
        station.lat[i] = 10.883438 + i * 0.0000123;
        station.lon[i] = (i == 3U ? -106.796019 : 106.796019) + i * 0.0000456;
    }
    unsigned rounds = 0U;
    while (exchange(frame) > 0U) rounds++;
    assert(RouteLink_Complete(&buoy) && buoy.have == 40U && buoy.rev == 7U && !buoy.loop);
    assert(rounds == 10U);                            /* 4 points a frame */
    assert(buoy.points[3].lon_e6 == -106795882L);
    RouteLink_BeaconField(&buoy, field, sizeof(field));
    assert(strcmp(field, "w=7:40:0") == 0);

    /* Not for this buoy, a repeat, out of order, a bad checksum or digit: refused. */
    assert(!RouteLink_OnFrame(&buoy, with_cs("RTE,PHAO-02,8,0,1,0,10.0,106.0"), "PHAO-01"));
    assert(!RouteLink_OnFrame(&buoy, with_cs("RTE,PHAO-01,7,36,40,0,10.0,106.0"), "PHAO-01"));
    assert(!RouteLink_OnFrame(&buoy, with_cs("RTE,PHAO-01,8,4,40,0,10.0,106.0"), "PHAO-01"));
    assert(!RouteLink_OnFrame(&buoy, with_cs("RTE,PHAO-01,8,0,301,0"), "PHAO-01"));
    assert(!RouteLink_OnFrame(&buoy, with_cs("RTE,PHAO-01,8,0,2,0,95.0,106.0"), "PHAO-01"));
    assert(!RouteLink_OnFrame(&buoy, "RTE,PHAO-01,8,0,1,0,10.0,106.0", "PHAO-01"));      /* no checksum */
    char corrupt[64];
    strcpy(corrupt, with_cs("RTE,PHAO-01,8,0,1,0,10.0,106.0"));
    corrupt[20] = '3';                                                                     /* 10.0 -> 30.0 */
    assert(!RouteLink_OnFrame(&buoy, corrupt, "PHAO-01"));
    assert(!RouteLink_OnFrame(&buoy, "id=PHAO-01,10.0,106.0", "PHAO-01"));
    assert(buoy.rev == 7U && buoy.have == 40U);

    /* To the Pi: 7 points a line, then a 2 s pause, until $HBT reports rev 7. */
    assert(RouteLink_NextPiLine(&buoy, 1U, 1000U, line, sizeof(line)));
    assert(strncmp(line, "1,7,0,40,0,10.883438;106.796019,10.883450;106.796065,", 53) == 0);
    assert(strlen(line) + strlen("$RTE,*XX\n") < 192U);
    assert(strstr(line, ",10.883463;106.796110,") != NULL);
    assert(strstr(line, "-106.795882") != NULL);
    unsigned lines = 1U;
    while (RouteLink_NextPiLine(&buoy, 2U, 1000U, line, sizeof(line))) lines++;
    assert(lines == 6U);                              /* 40 points = 5 lines of 7 and one of 5 */
    assert(strncmp(line, "2,7,35,40,0,", 12) == 0);
    assert(!RouteLink_NextPiLine(&buoy, 3U, 2999U, line, sizeof(line)));
    assert(RouteLink_NextPiLine(&buoy, 3U, 3000U, line, sizeof(line)));
    assert(strncmp(line, "3,7,0,40,", 9) == 0);
    RouteLink_SetPiRev(&buoy, 7U);
    assert(!RouteLink_NextPiLine(&buoy, 4U, 9000U, line, sizeof(line)));
    RouteLink_BeaconField(&buoy, field, sizeof(field));
    assert(strcmp(field, "w=7:40:1") == 0);
    route_ack_t ack;
    assert(RouteDownlink_ParseAck("7:40:1", &ack) && ack.pi == 1U);
    assert(RouteDownlink_Frame(&station, "PHAO-01", &ack, frame, sizeof(frame)) == 0U);   /* done */
    assert(RouteDownlink_Frame(&station, "PHAO-02", NULL, frame, sizeof(frame)) == 0U);   /* other buoy */

    /* A new revision replaces it; a later frame of it before the first one is refused. */
    station.rev = 8U;
    station.count = 0U;
    station.loop = true;
    assert(exchange(frame) > 0U && strcmp(frame, with_cs("RTE,PHAO-01,8,0,0,1")) == 0);
    assert(RouteLink_Complete(&buoy) && buoy.total == 0U && buoy.loop);
    assert(RouteLink_NextPiLine(&buoy, 5U, 9000U, line, sizeof(line)) && strcmp(line, "5,8,0,0,1") == 0);
    assert(exchange(frame) == 0U);

    assert(!RouteDownlink_ParseAck("0:1:0", &ack) && !RouteDownlink_ParseAck("3:301:0", &ack) &&
           !RouteDownlink_ParseAck("3:1", &ack) && !RouteDownlink_ParseAck("3:1:2", &ack));
    puts("route_link_test: ok");
    return 0;
}
