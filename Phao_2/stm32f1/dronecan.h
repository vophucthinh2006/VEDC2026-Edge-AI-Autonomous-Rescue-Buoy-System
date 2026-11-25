/* The part of DroneCAN a GPS needs: node-ID allocation and the Fix2 message. No HAL: host-tested. */
#ifndef DRONECAN_H
#define DRONECAN_H
#include <stdbool.h>
#include <stdint.h>

#define DRONECAN_OWN_NODE 100U
#define DRONECAN_GPS_NODE 50U   /* handed to the GPS when it asks for an ID */

typedef bool (*dronecan_send_t)(uint32_t id, const uint8_t *data, uint8_t len);

typedef struct {
    dronecan_send_t send;
    /* node-ID allocation */
    uint8_t uid[16], uid_len, known_uid[16];
    bool known;
    uint32_t uid_ms;
    uint8_t alloc_tid, status_tid;
    /* Fix2 reassembly */
    uint8_t buf[256], source, tid, toggle;
    uint16_t len;
    bool receiving;
    /* latest fix */
    bool fresh;
    uint8_t status, sats, gps_node;   /* status 0 none, 1 time only, 2 2D, 3 3D */
    int64_t lat_1e8, lon_1e8;
    uint32_t fixes, bad;
} dronecan_t;

/* True for the frames dronecan_rx uses (cheap enough for the receive interrupt). */
bool dronecan_wanted(uint32_t id);
void dronecan_rx(dronecan_t *d, uint32_t id, const uint8_t *data, uint8_t len, uint32_t now_ms);
void dronecan_status(dronecan_t *d, uint32_t uptime_s);
/* NMEA GGA of the latest fix, with CR LF; out holds at least 96 bytes. */
int dronecan_gga(const dronecan_t *d, char *out, unsigned size);
#endif
