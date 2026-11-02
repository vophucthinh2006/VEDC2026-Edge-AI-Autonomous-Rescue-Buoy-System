#pragma once

/* One person, one boat (docs/victim-claim.md): the station hears every boat, so it decides who looks after
 * a person. The first boat to report somebody owns them; any other boat that reports a person within
 * CLAIM_MERGE_M of them is told, in the ACK of that very report, to leave them alone and go on
 * (RouteDownlink_Ack, the skip point). The other boats' ACKs carry the owned and rescued people near them
 * in turn, so a boat leaves them alone before it even stops. A person is rescued when their owner's
 * confirm button is pressed near them (event C), or when the admin says so (the cloud's "claims").
 * Plain C, no ESP-IDF: the host test and the simulator use it as it is. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CLAIM_SLOTS        16U
#define CLAIM_MERGE_M      6.0         /* two reports this close are the same person (or the same group) */
#define CLAIM_NEAR_M       60.0        /* a boat is told of claimed people this close to it */
#define CLAIM_CONFIRM_M    10.0        /* the owner's confirm button this close to a person rescues them */
#define CLAIM_OWNED_MS     600000U     /* an owned person is left to the owner for 10 min after the last report */
#define CLAIM_RESCUED_MS   1800000U    /* a rescued person is left alone for 30 min */

typedef enum { CLAIM_FREE = 0, CLAIM_OWNED = 1, CLAIM_RESCUED = 2 } claim_status_t;

typedef struct {
    uint8_t status;            /* claim_status_t; CLAIM_FREE: the slot is empty */
    double lat, lon;
    char owner[32];
    uint32_t last_ms;          /* the last report of the person, or the cloud's word */
} claim_t;

typedef struct {
    claim_t c[CLAIM_SLOTS];
    uint32_t turn;             /* which of several claimed people the next ACK names */
} claims_t;

void Claims_Init(claims_t *claims);

/* A person reported by boat id at lat, lon (event V with a position). Returns the claim: a new one owned
 * by id, or the one already within CLAIM_MERGE_M (owner unchanged; refreshed when id is the owner).
 * NULL only when every slot is taken by a newer claim. */
const claim_t *Claims_OnReport(claims_t *claims, const char *id, double lat, double lon, uint32_t now_ms);

/* The confirm button of boat id at lat, lon (event C; has_position false: the boat's newest claim).
 * The person it owns there is rescued. Returns that claim or NULL. */
const claim_t *Claims_OnConfirm(claims_t *claims, const char *id, bool has_position, double lat, double lon, uint32_t now_ms);

/* The cloud's word on the person at lat, lon (GET /api/downlink "claims"): owner and status, or
 * CLAIM_FREE to let them go (another boat may take them). Creates the claim when there is none. */
void Claims_Set(claims_t *claims, double lat, double lon, const char *owner, claim_status_t status, uint32_t now_ms);

/* Lets go of every person boat id owns (the boat went home or was lost). */
void Claims_Release(claims_t *claims, const char *owner);

/* The one person boat id should leave alone, for its ACK: verdict first (the claim just reported, when
 * another boat owns it or it is rescued), else in turn each person within CLAIM_NEAR_M of the boat
 * (anywhere without has_position) that another boat owns or that is rescued. False: none. */
bool Claims_SkipFor(claims_t *claims, const char *id, const claim_t *verdict, bool has_position, double lat, double lon,
                    uint32_t now_ms, double *skip_lat, double *skip_lon);

/* Metres between two points (equirectangular, fine for a few hundred metres). */
double Claims_DistanceM(double lat1, double lon1, double lat2, double lon2);
