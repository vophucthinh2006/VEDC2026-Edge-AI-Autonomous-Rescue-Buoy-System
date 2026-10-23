#include "lora_payload.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int main(void) {
    lora_payload_t payload;
    const char *frame = "id=PHAO-01,10.762622,106.660172,r=2.1,p=-1.4,y=278.5,t=280.0,m=A,i=1,c=3,q=12";
    assert(strlen(frame) < 100U);
    assert(lora_payload_parse(frame, &payload));
    assert(strcmp(payload.id, "PHAO-01") == 0);
    assert(payload.has_position && payload.has_attitude);
    assert(fabs(payload.lat - 10.762622) < 0.000001);
    assert(payload.mode == 'A' && payload.imu_ok == 1U && payload.calibration == 3U && payload.sequence == 12U);
    assert(!payload.has_victims); /* firmware before the victim count */

    /* Longest frame the buoy can build, with the victim count. */
    frame = "id=PHAO-01,-10.762622,-106.660172,r=-179.9,p=-89.9,y=359.9,t=359.9,m=A,i=1,c=3,q=4294967295,v=255";
    assert(strlen(frame) < 100U);
    assert(lora_payload_parse(frame, &payload));
    assert(payload.has_position && payload.has_attitude && payload.has_victims && payload.victims == 255U);
    assert(lora_payload_parse("id=PHAO-01,NO_FIX,r=0.0,p=0.0,y=0.0,t=-1.0,m=S,i=0,c=0,q=13,v=2", &payload));
    assert(payload.has_attitude && payload.has_victims && payload.victims == 2U);
    assert(lora_payload_parse("id=PHAO-01,10.0,106.0,r=0,p=0,y=0,t=0,m=A,i=1,c=3,q=1,v=256", &payload));
    assert(payload.has_attitude && !payload.has_victims);

    assert(lora_payload_parse("id=PHAO-01,NO_FIX,r=0.0,p=0.0,y=0.0,t=-1.0,m=S,i=0,c=0,q=13", &payload));
    assert(!payload.has_position && payload.has_attitude && payload.target_yaw == -1.0f);

    assert(lora_payload_parse("id=OLD,10.0,106.0,seq=4", &payload));
    assert(payload.has_position && !payload.has_attitude); /* legacy compatibility */

    assert(!payload.attitude_rejected);

    /* A bad attitude group is dropped, the position is kept. */
    const char *bad[] = {
        "id=BAD,10.0,106.0,r=0,p=0,y=360,t=0,m=A,i=1,c=3,q=1",
        "id=BAD,10.0,106.0,r=181,p=0,y=0,t=0,m=A,i=1,c=3,q=1",
        "id=BAD,10.0,106.0,r=0,p=91,y=0,t=0,m=A,i=1,c=3,q=1",
        "id=BAD,10.0,106.0,r=0,p=0",
    };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        assert(lora_payload_parse(bad[i], &payload));
        assert(payload.has_position && !payload.has_attitude && payload.attitude_rejected);
        assert(fabs(payload.lat - 10.0) < 0.000001 && fabs(payload.lon - 106.0) < 0.000001);
    }
    assert(lora_payload_parse("id=BAD,NO_FIX,r=0,p=0,y=360,t=0,m=A,i=1,c=3,q=1", &payload));
    assert(!payload.has_position && !payload.has_attitude && payload.attitude_rejected);

    assert(!lora_payload_parse("", &payload));

    /* Route progress (docs/lora-route-downlink.md); the longest frame with it stays under 120 bytes. */
    frame = "id=PHAO-01,-10.762622,-106.660172,r=-179.9,p=-89.9,y=359.9,t=359.9,m=A,i=1,c=3,q=4294967295,v=255,w=65535:300:1";
    assert(strlen(frame) <= 120U);
    assert(lora_payload_parse(frame, &payload));
    assert(payload.has_attitude && payload.has_victims && payload.has_route);
    assert(payload.route_rev == 65535U && payload.route_have == 300U && payload.route_pi == 1U);
    assert(lora_payload_parse("id=PHAO-01,10.0,106.0,r=0,p=0,y=0,t=0,m=A,i=1,c=3,q=1,v=0,w=7:12:0", &payload));
    assert(payload.has_route && payload.route_rev == 7U && payload.route_have == 12U && payload.route_pi == 0U);
    const char *bad_route[] = { "w=0:1:0", "w=7:301:0", "w=7:1:2", "w=7:1", "w=x:1:0", "w=7::0" };
    for (size_t i = 0; i < sizeof(bad_route) / sizeof(bad_route[0]); i++) {
        char text[96];
        snprintf(text, sizeof(text), "id=PHAO-01,10.0,106.0,r=0,p=0,y=0,t=0,m=A,i=1,c=3,q=1,%s", bad_route[i]);
        assert(lora_payload_parse(text, &payload));
        assert(!payload.has_route && payload.has_attitude);
    }

    /* Course over ground, speed, battery, lights; R while driving home with the station lost. */
    frame = "id=PHAO-01,10.883438,106.796019,r=0.4,p=-1.0,y=54.2,t=55.0,m=R,i=1,c=3,q=120,v=2,w=7:12:1,L=3,b=87,g=52.1,s=0.25";
    assert(strlen(frame) < 120U);
    assert(lora_payload_parse(frame, &payload));
    assert(payload.has_attitude && payload.mode == 'R' && payload.has_route);
    assert(payload.has_lights && payload.lights == 3U && payload.has_battery && payload.battery_pct == 87U);
    assert(payload.has_course && fabsf(payload.course - 52.1f) < 0.01f && payload.has_speed && fabsf(payload.speed - 0.25f) < 0.001f);
    assert(lora_payload_parse("id=PHAO-01,NO_FIX,r=0,p=0,y=0,t=-1,m=S,i=0,c=0,q=1,L=4,b=101,g=360.0,s=-1", &payload));
    assert(payload.has_attitude && !payload.has_lights && !payload.has_battery && !payload.has_course && !payload.has_speed);
    assert(lora_payload_parse("id=PHAO-01,NO_FIX,r=0,p=0,y=0,t=-1,m=S,i=0,c=0,q=1,H=1,L=0", &payload));
    assert(payload.has_hold && payload.hold == 1U && !payload.has_route);
    assert(lora_payload_parse("id=PHAO-01,NO_FIX,r=0,p=0,y=0,t=-1,m=S,i=0,c=0,q=1,H=2", &payload) && !payload.has_hold);
    assert(lora_payload_parse("id=PHAO-01,NO_FIX,r=0,p=0,y=0,t=-1,m=X,i=0,c=0,q=1", &payload));
    assert(!payload.has_attitude);

    /* Event frames: a beacon is not one, and an event is not a beacon to the event parser. */
    lora_event_t ev;
    assert(!lora_event_parse("id=PHAO-01,10.0,106.0", &ev));
    assert(lora_event_parse("ev=PHAO-01,5,V,10.883500,106.796100,3,0.87,2", &ev));
    assert(strcmp(ev.id, "PHAO-01") == 0 && ev.seq == 5U && ev.type == 'V' && ev.has_position);
    assert(fabs(ev.lat - 10.8835) < 1e-6 && ev.risk == 3U && fabsf(ev.confidence - 0.87f) < 1e-4f && ev.count == 2U);
    assert(lora_event_parse("ev=PHAO-01,6,V,,,1,0.50,3", &ev) && !ev.has_position && ev.risk == 1U);
    assert(lora_event_parse("ev=PHAO-01,7,T,-10.5,-106.25,-120.5,4.0", &ev));
    assert(ev.type == 'T' && ev.has_position && fabsf(ev.roll + 120.5f) < 1e-3f && fabsf(ev.pitch - 4.0f) < 1e-3f);
    assert(lora_event_parse("ev=PHAO-01,8,C,,,4", &ev) && ev.type == 'C' && ev.presses == 4U && !ev.has_position);
    const char *bad_event[] = {
        "ev=PHAO-01,0,C,,,1",            /* eseq 0 is "no event" */
        "ev=PHAO-01,9,V,,,4,0.5,1",      /* risk past critical */
        "ev=PHAO-01,9,V,,,1,1.5,1",      /* confidence past 1 */
        "ev=PHAO-01,9,V,10.0,,1,0.5,1",  /* half a position */
        "ev=PHAO-01,9,T,,,1.0",          /* pitch missing */
        "ev=PHAO-01,9,X,,,1",
        "ev=,9,C,,,1",
    };
    for (size_t i = 0; i < sizeof(bad_event) / sizeof(bad_event[0]); i++) assert(!lora_event_parse(bad_event[i], &ev));

    /* Diagnostics frames (STM32F4/App/diag_frame.h), as the F407 builds them */
    lora_diag_t dg;
    assert(lora_diag_parse("dg=PHAO-01,n=obstacle_too_close,s=P,w=2/5,a=0,d=2,u=1184,j=-35,k=3:1", &dg));
    assert(strcmp(dg.id, "PHAO-01") == 0 && strcmp(dg.reason, "obstacle_too_close") == 0 && dg.mission == 'P');
    assert(dg.wp == 2U && dg.wp_total == 5U && dg.arm_block == 0U && dg.drive_block == 2U);
    assert(dg.has_battery && fabsf(dg.volts - 11.84f) < 1e-4f && fabsf(dg.amps + 0.35f) < 1e-4f);
    assert(dg.i2c_recoveries == 3U && dg.imu_reinits == 1U);
    /* Nothing known yet, no INA226 */
    assert(lora_diag_parse("dg=PHAO-01,n=-,s=-,w=0/0,a=1,d=1,k=0:0", &dg));
    assert(dg.reason[0] == '\0' && dg.mission == 0 && !dg.has_battery && dg.arm_block == 1U);
    assert(!dg.has_fps);
    /* The Pi's detector running: frames a second x10 */
    assert(lora_diag_parse("dg=PHAO-01,n=waypoint,s=P,w=1/3,a=0,d=0,k=0:0,f=13", &dg));
    assert(dg.has_fps && fabsf(dg.fps - 1.3f) < 1e-4f);
    assert(!lora_diag_parse("dg=PHAO-01,n=waypoint,s=P,w=1/3,a=0,d=0,k=0:0,f=1000", &dg));   /* out of range */
    /* A field of newer firmware is skipped */
    assert(lora_diag_parse("dg=PHAO-01,n=waypoint,s=B,w=1/3,a=0,d=0,k=0:0,z=7", &dg) && dg.mission == 'B');
    const char *bad_diag[] = {
        "dg=PHAO-01,n=waypoint,s=P,w=1/3,a=0,d=0",            /* k missing */
        "dg=PHAO-01,n=Way-point,s=P,w=1/3,a=0,d=0,k=0:0",     /* reason outside [a-z0-9_] */
        "dg=PHAO-01,n=waypoint,s=X,w=1/3,a=0,d=0,k=0:0",      /* unknown mission */
        "dg=PHAO-01,n=waypoint,s=P,w=1/3,a=3,d=0,k=0:0",      /* arm block past overturned */
        "dg=PHAO-01,n=waypoint,s=P,w=1/3,a=0,d=0,u=1184,k=0:0",  /* volts without amps */
        "dg=,n=waypoint,s=P,w=1/3,a=0,d=0,k=0:0",
        "id=PHAO-01,10.0,106.0",
    };
    for (size_t i = 0; i < sizeof(bad_diag) / sizeof(bad_diag[0]); i++) assert(!lora_diag_parse(bad_diag[i], &dg));
    /* A beacon is never taken for a diagnostics frame, nor the other way round */
    assert(!lora_diag_parse("id=PHAO-01,10.0,106.0,r=0,p=0,y=0,t=0,m=A,i=1,c=3,q=1", &dg));

    puts("lora_payload_test: PASS");
    return 0;
}
