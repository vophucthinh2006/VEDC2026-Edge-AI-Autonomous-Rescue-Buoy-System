#include "victim_claims.h"
#include <math.h>
#include <string.h>

#define EARTH_M 6371000.0
#define RAD     (3.14159265358979323846 / 180.0)

double Claims_DistanceM(double lat1, double lon1, double lat2, double lon2)
{
    double x = (lon2 - lon1) * RAD * cos((lat1 + lat2) * 0.5 * RAD), y = (lat2 - lat1) * RAD;
    return EARTH_M * sqrt(x * x + y * y);
}

void Claims_Init(claims_t *claims)
{
    memset(claims, 0, sizeof(*claims));
}

static bool expired(const claim_t *c, uint32_t now_ms)
{
    uint32_t keep = c->status == CLAIM_RESCUED ? CLAIM_RESCUED_MS : CLAIM_OWNED_MS;
    return c->status != CLAIM_FREE && (uint32_t)(now_ms - c->last_ms) >= keep;
}

static void sweep(claims_t *claims, uint32_t now_ms)
{
    for (size_t i = 0U; i < CLAIM_SLOTS; i++)
        if (expired(&claims->c[i], now_ms)) claims->c[i].status = CLAIM_FREE;
}

/* The claim within CLAIM_MERGE_M of lat, lon, the nearest when several */
static claim_t *near(claims_t *claims, double lat, double lon)
{
    claim_t *best = NULL;
    double best_d = CLAIM_MERGE_M;
    for (size_t i = 0U; i < CLAIM_SLOTS; i++) {
        claim_t *c = &claims->c[i];
        if (c->status == CLAIM_FREE) continue;
        double d = Claims_DistanceM(lat, lon, c->lat, c->lon);
        if (d <= best_d) { best = c; best_d = d; }
    }
    return best;
}

/* A free slot, else the oldest owned one (rescued people are worth keeping longer) */
static claim_t *slot(claims_t *claims)
{
    claim_t *oldest = NULL;
    for (size_t i = 0U; i < CLAIM_SLOTS; i++) {
        claim_t *c = &claims->c[i];
        if (c->status == CLAIM_FREE) return c;
        if (c->status == CLAIM_OWNED && (oldest == NULL || (int32_t)(c->last_ms - oldest->last_ms) < 0)) oldest = c;
    }
    return oldest;
}

static void set_owner(claim_t *c, const char *owner)
{
    size_t n = strlen(owner);
    if (n >= sizeof(c->owner)) n = sizeof(c->owner) - 1U;
    memcpy(c->owner, owner, n);
    c->owner[n] = '\0';
}

const claim_t *Claims_OnReport(claims_t *claims, const char *id, double lat, double lon, uint32_t now_ms)
{
    sweep(claims, now_ms);
    claim_t *c = near(claims, lat, lon);
    if (c != NULL) {
        if (c->status == CLAIM_OWNED && strcmp(c->owner, id) == 0) { c->lat = lat; c->lon = lon; c->last_ms = now_ms; }
        return c;
    }
    c = slot(claims);
    if (c == NULL) return NULL;
    c->status = CLAIM_OWNED;
    c->lat = lat;
    c->lon = lon;
    set_owner(c, id);
    c->last_ms = now_ms;
    return c;
}

const claim_t *Claims_OnConfirm(claims_t *claims, const char *id, bool has_position, double lat, double lon, uint32_t now_ms)
{
    sweep(claims, now_ms);
    claim_t *best = NULL;
    double best_d = CLAIM_CONFIRM_M;
    for (size_t i = 0U; i < CLAIM_SLOTS; i++) {
        claim_t *c = &claims->c[i];
        if (c->status != CLAIM_OWNED || strcmp(c->owner, id) != 0) continue;
        if (has_position) {
            double d = Claims_DistanceM(lat, lon, c->lat, c->lon);
            if (d <= best_d) { best = c; best_d = d; }
        } else if (best == NULL || (int32_t)(c->last_ms - best->last_ms) > 0) {
            best = c;
        }
    }
    if (best != NULL) { best->status = CLAIM_RESCUED; best->last_ms = now_ms; }
    return best;
}

void Claims_Set(claims_t *claims, double lat, double lon, const char *owner, claim_status_t status, uint32_t now_ms)
{
    sweep(claims, now_ms);
    claim_t *c = near(claims, lat, lon);
    if (status == CLAIM_FREE) {
        if (c != NULL) c->status = CLAIM_FREE;
        return;
    }
    if (c == NULL && (c = slot(claims)) == NULL) return;
    c->status = (uint8_t)status;
    c->lat = lat;
    c->lon = lon;
    set_owner(c, owner != NULL ? owner : "");
    c->last_ms = now_ms;
}

void Claims_Release(claims_t *claims, const char *owner)
{
    for (size_t i = 0U; i < CLAIM_SLOTS; i++)
        if (claims->c[i].status == CLAIM_OWNED && strcmp(claims->c[i].owner, owner) == 0) claims->c[i].status = CLAIM_FREE;
}

/* Whether boat id should leave c alone: rescued, or owned by another boat */
static bool not_for(const claim_t *c, const char *id)
{
    return c->status == CLAIM_RESCUED || (c->status == CLAIM_OWNED && strcmp(c->owner, id) != 0);
}

bool Claims_SkipFor(claims_t *claims, const char *id, const claim_t *verdict, bool has_position, double lat, double lon,
                    uint32_t now_ms, double *skip_lat, double *skip_lon)
{
    sweep(claims, now_ms);
    if (verdict != NULL && not_for(verdict, id)) {
        *skip_lat = verdict->lat;
        *skip_lon = verdict->lon;
        return true;
    }
    for (size_t k = 0U; k < CLAIM_SLOTS; k++) {
        const claim_t *c = &claims->c[(claims->turn + k) % CLAIM_SLOTS];
        if (!not_for(c, id)) continue;
        if (has_position && Claims_DistanceM(lat, lon, c->lat, c->lon) > CLAIM_NEAR_M) continue;
        claims->turn = (uint32_t)((claims->turn + k + 1U) % CLAIM_SLOTS);
        *skip_lat = c->lat;
        *skip_lon = c->lon;
        return true;
    }
    return false;
}
