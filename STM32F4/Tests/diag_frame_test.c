/* Host test of App/diag_frame.c: the diagnostics frame for the dashboard. */
#include "diag_frame.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    char buf[120];
    diag_t d = { 0 };

    /* Nothing known yet: dashes and zeros, no battery. */
    assert(DiagFrame_Build(buf, sizeof(buf), "PHAO-01", &d) > 0U);
    assert(strcmp(buf, "dg=PHAO-01,n=-,s=-,w=0/0,a=0,d=0,k=0:0") == 0);

    /* A boat stopped at an obstacle, its pack read. */
    Diag_SetReason(&d, "obstacle_too_close");
    d.mission = 'P'; d.wp = 2; d.wp_total = 5;
    d.arm_block = 0; d.drive_block = 2;
    d.has_battery = true; d.volts = 11.84f; d.amps = -0.35f;
    d.i2c_recoveries = 3; d.imu_reinits = 1;
    size_t n = DiagFrame_Build(buf, sizeof(buf), "PHAO-01", &d);
    assert(strcmp(buf, "dg=PHAO-01,n=obstacle_too_close,s=P,w=2/5,a=0,d=2,u=1184,j=-35,k=3:1") == 0);
    assert(n == strlen(buf));

    /* The Pi heard: its reason, mission and waypoint. Silent (rebooted on the stand, 2026-10-22):
       the last reason it gave no longer holds, nor its mission. */
    diag_t p = { 0 };
    Diag_SetPi(&p, true, "leaving_obstacle", 'P', 1U, 4U);
    assert(strcmp(p.reason, "leaving_obstacle") == 0 && p.mission == 'P' && p.wp == 1U && p.wp_total == 4U);
    p.drive_block = 2U;
    Diag_SetPi(&p, false, "leaving_obstacle", 'P', 1U, 4U);
    assert(DiagFrame_Build(buf, sizeof(buf), "PHAO-01", &p) > 0U);
    assert(strcmp(buf, "dg=PHAO-01,n=pi_silent,s=-,w=0/0,a=0,d=2,k=0:0") == 0);

    /* The Pi's detector running: its frames a second x10 last. */
    p.has_fps = true; p.fps_x10 = 13U;
    assert(DiagFrame_Build(buf, sizeof(buf), "PHAO-01", &p) > 0U);
    assert(strcmp(buf, "dg=PHAO-01,n=pi_silent,s=-,w=0/0,a=0,d=2,k=0:0,f=13") == 0);

    /* The longest reason and every field at its widest still fit the 120-byte LoRa payload. */
    Diag_SetReason(&d, "stm32_fault_or_estop_and_more_text");
    assert(strlen(d.reason) == DIAG_REASON_MAX - 1U);
    d.wp = 300; d.wp_total = 300; d.volts = 25.2f; d.amps = -80.0f; d.i2c_recoveries = 65535; d.imu_reinits = 65535;
    d.has_fps = true; d.fps_x10 = 999U;
    assert(DiagFrame_Build(buf, sizeof(buf), "PHAO-01", &d) > 0U && strlen(buf) < 120U);

    /* Only [a-z0-9_] reaches the frame: no comma or '=' can break the fields. */
    Diag_SetReason(&d, "bad,reason=x Y");
    assert(strcmp(d.reason, "badreasonx") == 0);

    /* An unknown mission letter is sent as unknown. */
    d.mission = 'Z';
    DiagFrame_Build(buf, sizeof(buf), "PHAO-01", &d);
    assert(strstr(buf, ",s=-,") != NULL);

    /* Too small a buffer: nothing, never a cut frame. */
    assert(DiagFrame_Build(buf, 20U, "PHAO-01", &d) == 0U && buf[0] == '\0');

    printf("diag_frame_test: PASS\n");
    return 0;
}
