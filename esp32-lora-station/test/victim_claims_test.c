/* Host test of main/victim_claims.c: the first boat to report a person owns them, the others are told to
 * leave them alone; the owner's confirm button rescues them; the cloud's word and a boat called home let go. */
#include "victim_claims.h"
#include "route_downlink.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define LAT 10.883438
#define LON 106.796019
#define M_LAT (1.0 / 111195.0)   /* one metre north, in degrees */
#define STARTS(frame, lit) (strncmp((frame), (lit), sizeof(lit) - 1U) == 0)

int main(void)
{
    claims_t claims;
    double sl, so;
    Claims_Init(&claims);

    /* PHAO-01 first: it owns the person and is not told to skip them. */
    const claim_t *c = Claims_OnReport(&claims, "PHAO-01", LAT, LON, 1000U);
    assert(c && c->status == CLAIM_OWNED && strcmp(c->owner, "PHAO-01") == 0);
    assert(!Claims_SkipFor(&claims, "PHAO-01", c, true, LAT, LON, 1000U, &sl, &so));

    /* PHAO-02 reports the same person 4 m off: the same claim, still PHAO-01's; the verdict goes to PHAO-02. */
    const claim_t *v = Claims_OnReport(&claims, "PHAO-02", LAT + 4.0 * M_LAT, LON, 2000U);
    assert(v == c && strcmp(v->owner, "PHAO-01") == 0 && fabs(v->lat - LAT) < 1e-9);
    assert(Claims_SkipFor(&claims, "PHAO-02", v, true, LAT, LON, 2000U, &sl, &so) && sl == LAT && so == LON);

    /* 10 m off is somebody else: PHAO-02 owns them. */
    const claim_t *d = Claims_OnReport(&claims, "PHAO-02", LAT + 10.0 * M_LAT, LON, 3000U);
    assert(d && d != c && strcmp(d->owner, "PHAO-02") == 0);

    /* PHAO-03 near both is told of each in turn by its beacons' ACKs; far away (100 m) of none. */
    bool seen1 = false, seen2 = false;
    for (int i = 0; i < 4; i++) {
        assert(Claims_SkipFor(&claims, "PHAO-03", NULL, true, LAT, LON, 4000U, &sl, &so));
        if (sl == LAT) seen1 = true; else seen2 = true;
    }
    assert(seen1 && seen2);
    assert(!Claims_SkipFor(&claims, "PHAO-03", NULL, true, LAT + 100.0 * M_LAT, LON, 4000U, &sl, &so));
    /* Without its own position: any of them. */
    assert(Claims_SkipFor(&claims, "PHAO-03", NULL, false, 0.0, 0.0, 4000U, &sl, &so));
    /* PHAO-01 is only told of PHAO-02's person. */
    for (int i = 0; i < 3; i++) {
        assert(Claims_SkipFor(&claims, "PHAO-01", NULL, true, LAT, LON, 4000U, &sl, &so));
        assert(fabs(sl - (LAT + 10.0 * M_LAT)) < 1e-9);
    }

    /* PHAO-02's button far from PHAO-01's person does nothing to it; near its own person rescues them. */
    assert(Claims_OnConfirm(&claims, "PHAO-02", true, LAT - 5.0 * M_LAT, LON, 5000U) == NULL);
    const claim_t *r = Claims_OnConfirm(&claims, "PHAO-02", true, LAT + 9.0 * M_LAT, LON, 5000U);
    assert(r == d && r->status == CLAIM_RESCUED);
    /* A rescued person is skipped by everybody, the owner too, and a new report of them changes nothing. */
    assert(Claims_SkipFor(&claims, "PHAO-02", Claims_OnReport(&claims, "PHAO-02", LAT + 10.0 * M_LAT, LON, 6000U),
                          true, LAT, LON, 6000U, &sl, &so) && fabs(sl - (LAT + 10.0 * M_LAT)) < 1e-9);
    assert(d->status == CLAIM_RESCUED && strcmp(d->owner, "PHAO-02") == 0);
    /* The button without a position: the boat's newest owned person. */
    assert(Claims_OnConfirm(&claims, "PHAO-01", false, 0.0, 0.0, 6500U) == c && c->status == CLAIM_RESCUED);

    /* The cloud lets PHAO-01's person go (admin "Nhả"): the next boat to report them owns them. */
    Claims_Set(&claims, LAT, LON, "PHAO-01", CLAIM_FREE, 7000U);
    assert(c->status == CLAIM_FREE);
    c = Claims_OnReport(&claims, "PHAO-03", LAT, LON, 7500U);
    assert(c && strcmp(c->owner, "PHAO-03") == 0 && c->status == CLAIM_OWNED);
    /* The cloud names an owner the station had not heard of. */
    Claims_Set(&claims, LAT + 30.0 * M_LAT, LON, "PHAO-02", CLAIM_OWNED, 8000U);
    v = Claims_OnReport(&claims, "PHAO-01", LAT + 30.0 * M_LAT, LON, 8100U);
    assert(v && strcmp(v->owner, "PHAO-02") == 0);

    /* PHAO-03 called home: its people go free. */
    Claims_Release(&claims, "PHAO-03");
    assert(c->status == CLAIM_FREE);

    /* An owned person is left to the owner for CLAIM_OWNED_MS after the last report, then goes free. */
    assert(v->status == CLAIM_OWNED);
    assert(!Claims_SkipFor(&claims, "PHAO-01", NULL, true, LAT + 100.0 * M_LAT, LON, 8000U + CLAIM_OWNED_MS, &sl, &so));
    assert(v->status == CLAIM_FREE);

    /* Full: the oldest owned claim gives way, a rescued one does not. */
    Claims_Init(&claims);
    Claims_Set(&claims, LAT, LON, "PHAO-01", CLAIM_RESCUED, 0U);
    for (unsigned i = 1U; i <= CLAIM_SLOTS; i++)
        assert(Claims_OnReport(&claims, "PHAO-02", LAT + 20.0 * i * M_LAT, LON, i * 10U) != NULL);
    assert(Claims_SkipFor(&claims, "PHAO-03", NULL, true, LAT, LON, 1000U, &sl, &so));
    int rescued = 0;
    for (unsigned i = 0U; i < CLAIM_SLOTS; i++) rescued += claims.c[i].status == CLAIM_RESCUED;
    assert(rescued == 1);

    /* The ACK with a skip point: eleven fields, the formation ones empty outside a formation. */
    char frame[DOWNLINK_FRAME_MAX + 1U];
    size_t n = RouteDownlink_AckSkip("PHAO-02", 7U, 0U, true, 10.883438, 106.796019, NULL, true, 10.883500, 106.796100,
                                     frame, sizeof(frame));
    assert(n == strlen(frame) && STARTS(frame, "ACK,PHAO-02,7,0,10.883438,106.796019,,,,10.883500,106.796100*"));
    formation_role_t role = { .on = true, .leader = "PHAO-01", .right_m = -12, .back_m = 0 };
    n = RouteDownlink_AckSkip("PHAO-02", 0U, 0U, true, 10.883438, 106.796019, &role, true, 10.883500, 106.796100,
                              frame, sizeof(frame));
    assert(STARTS(frame, "ACK,PHAO-02,0,0,10.883438,106.796019,PHAO-01,-12,0,10.883500,106.796100*"));
    assert(n <= DOWNLINK_FRAME_MAX);
    /* No skip: the frame older firmware takes. */
    RouteDownlink_AckSkip("PHAO-02", 0U, 0U, true, 10.883438, 106.796019, NULL, false, 0.0, 0.0, frame, sizeof(frame));
    assert(STARTS(frame, "ACK,PHAO-02,0,0,10.883438,106.796019*"));

    puts("victim_claims_test: ok");
    return 0;
}
