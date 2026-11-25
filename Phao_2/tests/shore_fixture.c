/* Run the REAL shore parser, claim arbiter and ACK serializer against a Pi frame. */
#include "lora_payload.h"
#include "victim_claims.h"
#include "route_downlink.h"
#include <stdio.h>
#include <string.h>
int main(int argc, char **argv) {
    if (argc != 3) return 1;
    if (!strcmp(argv[1], "diag")) {
        lora_diag_t dg;
        if (!lora_diag_parse(argv[2], &dg)) return 4;
        printf("%s %s fps=%d\n", dg.id, dg.reason, dg.has_fps ? (int)(dg.fps * 10.0f + 0.5f) : -1);
        return 0;
    }
    lora_event_t event;
    /* risk 0 = safe: the dashboard would not raise a rescue alert. */
    if (!lora_event_parse(argv[2], &event) || event.type != 'V' || !event.has_position || event.risk == 0) return 2;
    claims_t claims; Claims_Init(&claims);
    if (!strcmp(argv[1], "existing"))
        Claims_OnReport(&claims, "PHAO-01", event.lat, event.lon, 1000);
    const claim_t *verdict = Claims_OnReport(&claims, event.id, event.lat, event.lon, 2000);
    double lat=0, lon=0;
    bool skip = Claims_SkipFor(&claims, event.id, verdict, false, 0, 0, 2000, &lat, &lon);
    char ack[160];
    if (!RouteDownlink_AckSkip(event.id,event.seq,0,false,0,0,NULL,skip,lat,lon,ack,sizeof(ack))) return 3;
    puts(ack);
    return 0;
}
