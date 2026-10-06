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

    puts("lora_payload_test: PASS");
    return 0;
}
