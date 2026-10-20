/* Host test of Modules/GPS/gps_nmea.c: GGA with and without a fix, RMC course and speed, bad checksums. */
#include "gps_nmea.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void feed(gps_state_t *gps, const char *body, uint32_t now)
{
    unsigned cs = 0U;
    for (const char *p = body + 1; *p != '\0'; p++) cs ^= (unsigned char)*p;
    char line[160];
    snprintf(line, sizeof(line), "%s*%02X\r\n", body, cs);
    for (const char *p = line; *p != '\0'; p++) GPS_FeedByte(gps, (uint8_t)*p, now);
}

int main(void)
{
    gps_state_t gps;
    GPS_Init(&gps);

    /* Indoors, no fix: the GPS is talking, last_rx_ms says so, nothing else changes. */
    feed(&gps, "$GNGGA,,,,,,0,00,99.99,,,,,,", 1000U);
    assert(gps.last_rx_ms == 1000U && gps.fix == 0U && !gps.valid && gps.sentences == 1U);
    assert(gps.bytes > 20U);

    /* A fix. */
    feed(&gps, "$GNGGA,061530.00,1053.00628,N,10647.76114,E,1,09,0.92,12.3,M,-2.1,M,,", 2000U);
    assert(gps.valid && gps.fix == 1U && fabsf(gps.hdop - 0.92f) < 1e-4f);
    assert(fabsf(gps.latitude_deg - 10.883438f) < 1e-5f && fabsf(gps.longitude_deg - 106.796019f) < 1e-5f);

    /* RMC: 1.2 knots heading 52.1 deg; standing still the course stays. */
    feed(&gps, "$GNRMC,061531.00,A,1053.00628,N,10647.76114,E,1.200,52.10,201026,,,A,V", 2100U);
    assert(fabsf(gps.speed_mps - 0.617f) < 0.001f && fabsf(gps.course_deg - 52.1f) < 1e-3f);
    feed(&gps, "$GPRMC,061532.00,A,1053.00628,N,10647.76114,E,0.010,,201026,,,A,V", 2200U);
    assert(gps.speed_mps < 0.01f && fabsf(gps.course_deg - 52.1f) < 1e-3f);
    /* RMC without a valid fix (V) changes nothing. */
    feed(&gps, "$GNRMC,061533.00,V,,,,,,,201026,,,N,V", 2300U);
    assert(fabsf(gps.course_deg - 52.1f) < 1e-3f);

    /* A wrong checksum is dropped; the fix is lost, then the position is kept. */
    const char *bad = "$GNGGA,061534.00,0000.00000,N,00000.00000,E,1,09,0.92,12.3,M,-2.1,M,,*00\r\n";
    for (const char *p = bad; *p != '\0'; p++) GPS_FeedByte(&gps, (uint8_t)*p, 2400U);
    assert(gps.bad_checksum == 1U && fabsf(gps.latitude_deg - 10.883438f) < 1e-5f);
    feed(&gps, "$GNGGA,061535.00,,,,,0,03,,,,,,,", 2500U);
    assert(!gps.valid && gps.hdop == 99.0f && fabsf(gps.latitude_deg - 10.883438f) < 1e-5f);

    /* A sentence cut short by an overrun: the next '$' starts clean. */
    for (const char *p = "$GNGGA,0615"; *p != '\0'; p++) GPS_FeedByte(&gps, (uint8_t)*p, 2600U);
    feed(&gps, "$GNGGA,061536.00,1053.00628,S,10647.76114,W,2,09,1.0,1,M,0,M,,", 2600U);
    assert(gps.fix == 2U && gps.latitude_deg < 0.0f && gps.longitude_deg < 0.0f);

    printf("gps_nmea_test: PASS\n");
    return 0;
}
