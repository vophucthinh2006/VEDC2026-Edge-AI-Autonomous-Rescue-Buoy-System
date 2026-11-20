#include "uart_protocol.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Feeds "$body*CS\n" as the Pi sends it. */
static void feed(pi_command_t *c, const char *body, uint32_t now_ms) {
    uint8_t cs = 0U;
    for (const char *p = body; *p; p++) cs ^= (uint8_t)*p;
    char line[200];
    snprintf(line, sizeof(line), "$%s*%02X\n", body, cs);
    for (const char *p = line; *p; p++) Protocol_FeedByte(c, (uint8_t)*p, now_ms);
}

static int near(float a, float b) { return fabsf(a - b) < 1e-4f; }

int main(void) {
    pi_command_t c;
    Protocol_Init(&c);

    /* $MAN from the phone: full range, the front astern. */
    feed(&c, "MAN,1,1.00,-0.50,0.40,3,300", 100U);
    assert(c.mode == NAV_MANUAL);
    assert(near(c.man_rear, 1.0f) && near(c.man_front, -0.5f) && near(c.man_steer, 0.4f));
    assert(c.man_lights == 3U && c.ttl_ms == 300U && c.last_nav_ms == 100U);

    /* Out of range: refused, the last good one stays. */
    feed(&c, "MAN,2,1.50,0.00,0.00,0,300", 200U);
    feed(&c, "MAN,3,0.50,0.00,-1.20,0,300", 200U);
    feed(&c, "MAN,4,0.50,0.00,0.00,4,300", 200U);
    feed(&c, "MAN,5,0.50,0.00,0.00,0,0", 200U);
    assert(near(c.man_rear, 1.0f) && c.last_nav_ms == 100U);

    /* An older sequence while the stream is live: dropped. */
    feed(&c, "MAN,10,0.80,0.00,0.00,0,300", 250U);
    feed(&c, "MAN,9,0.20,0.20,0.00,0,300", 260U);
    assert(near(c.man_rear, 0.8f));

    /* Back to AUTO: a NAV clears the manual throttles, so a NAV MANUAL can never drive on stale ones. */
    feed(&c, "NAV,11,AUTO,0.30,90.0,300", 300U);
    assert(c.mode == NAV_AUTO && near(c.man_rear, 0.0f) && near(c.man_front, 0.0f) && near(c.man_steer, 0.0f));
    feed(&c, "NAV,12,MANUAL,0.30,90.0,300", 400U);
    assert(c.mode == NAV_MANUAL && near(c.man_rear, 0.0f) && near(c.man_front, 0.0f));

    /* $CAM: tilt is optional. An older Pi's pan alone leaves the start tilt, a tilt is clamped to 0..60. */
    pi_command_t cam;
    Protocol_Init(&cam);
    assert(near(cam.cam_tilt_deg, 10.0f));
    feed(&cam, "CAM,1,-30.0", 100U);
    assert(near(cam.cam_pan_deg, -30.0f) && near(cam.cam_tilt_deg, 10.0f) && cam.last_cam_ms == 100U);
    feed(&cam, "CAM,2,20.0,30.0", 150U);
    assert(near(cam.cam_pan_deg, 20.0f) && near(cam.cam_tilt_deg, 30.0f));
    feed(&cam, "CAM,3,0.0,75.0", 200U);
    assert(near(cam.cam_tilt_deg, 60.0f));
    feed(&cam, "CAM,4,0.0,-8.0", 250U);
    assert(near(cam.cam_tilt_deg, 0.0f));
    feed(&cam, "CAM,5,5.0", 300U);
    assert(near(cam.cam_pan_deg, 5.0f) && near(cam.cam_tilt_deg, 0.0f));   /* pan alone keeps the last tilt */
    feed(&cam, "CAM,6,95.0,30.0", 350U);
    assert(near(cam.cam_pan_deg, 5.0f) && near(cam.cam_tilt_deg, 0.0f));   /* a pan out of range drops the packet */

    puts("uart_protocol_test OK");
    return 0;
}
