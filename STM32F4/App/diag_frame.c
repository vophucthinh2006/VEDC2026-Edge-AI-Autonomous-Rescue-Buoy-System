#include "diag_frame.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

void Diag_SetReason(diag_t *diag, const char *text)
{
    size_t n = 0U;
    for (const char *p = text; p != NULL && *p != '\0' && n < DIAG_REASON_MAX - 1U; p++) {
        char c = *p;
        if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_') diag->reason[n++] = c;
    }
    diag->reason[n] = '\0';
}

void Diag_SetPi(diag_t *diag, bool heard, const char *reason, char mission, uint16_t wp, uint16_t wp_total)
{
    if (!heard) {
        Diag_SetReason(diag, "pi_silent");
        diag->mission = 0;
        diag->wp = diag->wp_total = 0U;
        return;
    }
    Diag_SetReason(diag, reason);
    diag->mission = mission;
    diag->wp = wp;
    diag->wp_total = wp_total;
}

static bool mission_known(char m)
{
    return m == 'I' || m == 'P' || m == 'F' || m == 'R' || m == 'B' || m == 'H';
}

size_t DiagFrame_Build(char *buf, size_t size, const char *buoy_id, const diag_t *diag)
{
    if (size == 0U) return 0U;
    int n = snprintf(buf, size, "dg=%s,n=%s,s=%c,w=%u/%u,a=%u,d=%u", buoy_id,
                     diag->reason[0] != '\0' ? diag->reason : "-", mission_known(diag->mission) ? diag->mission : '-',
                     (unsigned)diag->wp, (unsigned)diag->wp_total, (unsigned)diag->arm_block, (unsigned)diag->drive_block);
    if (n < 0 || (size_t)n >= size) { buf[0] = '\0'; return 0U; }
    if (diag->has_battery) {
        int m = snprintf(buf + n, size - (size_t)n, ",u=%ld,j=%ld", lroundf(diag->volts * 100.0f), lroundf(diag->amps * 100.0f));
        if (m < 0 || (size_t)(n + m) >= size) { buf[0] = '\0'; return 0U; }
        n += m;
    }
    int m = snprintf(buf + n, size - (size_t)n, ",k=%u:%u", (unsigned)diag->i2c_recoveries, (unsigned)diag->imu_reinits);
    if (m < 0 || (size_t)(n + m) >= size) { buf[0] = '\0'; return 0U; }
    n += m;
    if (diag->has_fps) {
        m = snprintf(buf + n, size - (size_t)n, ",f=%u", (unsigned)diag->fps_x10);
        if (m < 0 || (size_t)(n + m) >= size) { buf[0] = '\0'; return 0U; }
        n += m;
    }
    return (size_t)n;
}
