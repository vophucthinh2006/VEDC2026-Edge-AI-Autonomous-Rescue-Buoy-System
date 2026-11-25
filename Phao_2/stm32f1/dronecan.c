#include "dronecan.h"
#include <stdio.h>
#include <string.h>

#define ALLOCATION 1U
#define NODE_STATUS 341U
#define FIX2 1063U
#define ALLOCATION_SIGNATURE 0x0b2a812620a11d40ULL
#define FIX2_SIGNATURE 0xca41e7000f37435fULL
#define SOT 0x80U
#define EOT 0x40U
#define TOGGLE 0x20U

static uint32_t source(uint32_t id) { return id & 0x7FU; }
static bool service(uint32_t id) { return (id & 0x80U) != 0U; }
static uint32_t type(uint32_t id) { return (id >> 8) & 0xFFFFU; }
/* An anonymous frame carries only the two low bits of its type. */
static bool is_allocation(uint32_t id) { return !service(id) && source(id) == 0U && (type(id) & 3U) == ALLOCATION; }
static bool is_fix2(uint32_t id) { return !service(id) && source(id) != 0U && type(id) == FIX2; }

bool dronecan_wanted(uint32_t id) { return is_allocation(id) || is_fix2(id); }

static uint16_t crc_add(uint16_t crc, uint8_t byte)
{
    crc ^= (uint16_t)(byte << 8);
    for (int i = 0; i < 8; i++) crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000U) ? 0x1021U : 0U));
    return crc;
}

static uint16_t transfer_crc(uint64_t signature, const uint8_t *data, unsigned len)
{
    uint16_t crc = 0xFFFFU;
    for (int i = 0; i < 8; i++) crc = crc_add(crc, (uint8_t)(signature >> (8 * i)));
    for (unsigned i = 0; i < len; i++) crc = crc_add(crc, data[i]);
    return crc;
}

static void broadcast(dronecan_t *d, uint32_t dtid, uint64_t signature, uint32_t priority, uint8_t *tid,
                      const uint8_t *data, unsigned len)
{
    uint32_t id = (priority << 24) | (dtid << 8) | DRONECAN_OWN_NODE;
    uint8_t bytes[2 + 17], total = (uint8_t)len, at = 0, toggle = 0;
    bool multi = len > 7U;
    if (multi) {
        uint16_t crc = transfer_crc(signature, data, len);
        bytes[0] = (uint8_t)crc; bytes[1] = (uint8_t)(crc >> 8); total = (uint8_t)(len + 2U);
    }
    memcpy(bytes + (multi ? 2 : 0), data, len);
    do {
        uint8_t frame[8], n = (uint8_t)(total - at > 7 ? 7 : total - at);
        memcpy(frame, bytes + at, n);
        frame[n] = (uint8_t)((at == 0 ? SOT : 0U) | (at + n == total ? EOT : 0U) | toggle | *tid);
        if (!d->send(id, frame, (uint8_t)(n + 1U))) break;
        at = (uint8_t)(at + n); toggle ^= TOGGLE;
    } while (at < total);
    *tid = (uint8_t)((*tid + 1U) & 31U);
}

void dronecan_status(dronecan_t *d, uint32_t uptime_s)
{
    uint8_t data[7] = { (uint8_t)uptime_s, (uint8_t)(uptime_s >> 8), (uint8_t)(uptime_s >> 16), (uint8_t)(uptime_s >> 24) };
    broadcast(d, NODE_STATUS, 0, 16, &d->status_tid, data, sizeof(data));
}

/* A node without an ID sends its 16-byte unique ID in pieces of 6, 6 and 4; each reply echoes what
   was received so far, the last one with the node ID. */
static void allocation(dronecan_t *d, const uint8_t *data, uint8_t len, uint32_t now_ms)
{
    if (len < 3U || (data[len - 1U] & (SOT | EOT | TOGGLE)) != (SOT | EOT)) return;
    uint8_t chunk = (uint8_t)(len - 2U);
    if (data[0] & 1U) d->uid_len = 0;
    else if (d->uid_len == 0U || now_ms - d->uid_ms > 500U) { d->uid_len = 0; return; }
    if (d->uid_len + chunk > 16U) { d->uid_len = 0; return; }
    memcpy(d->uid + d->uid_len, data + 1, chunk);
    d->uid_len = (uint8_t)(d->uid_len + chunk); d->uid_ms = now_ms;
    uint8_t reply[17] = {0};
    if (d->uid_len == 16U) {
        /* shortcut: one allocated node (the GPS), a table when a second DroneCAN device is added. */
        if (!d->known) { memcpy(d->known_uid, d->uid, 16); d->known = true; }
        if (memcmp(d->known_uid, d->uid, 16) != 0) { d->uid_len = 0; return; }
        reply[0] = (uint8_t)(DRONECAN_GPS_NODE << 1);
    }
    memcpy(reply + 1, d->uid, d->uid_len);
    broadcast(d, ALLOCATION, ALLOCATION_SIGNATURE, 30, &d->alloc_tid, reply, d->uid_len + 1U);
    if (d->uid_len == 16U) d->uid_len = 0;
}

/* DSDL: bits from the top of each byte, whole bytes little-endian, the odd bits last and highest. */
static uint64_t bits(const uint8_t *p, unsigned offset, unsigned count)
{
    uint64_t value = 0;
    for (unsigned shift = 0; count > 0U; shift += 8U) {
        unsigned take = count < 8U ? count : 8U;
        uint8_t byte = 0;
        for (unsigned i = 0; i < take; i++, offset++) byte = (uint8_t)((byte << 1) | ((p[offset / 8U] >> (7U - offset % 8U)) & 1U));
        value |= (uint64_t)byte << shift; count -= take;
    }
    return value;
}

static int64_t signed37(const uint8_t *p, unsigned offset)
{
    uint64_t v = bits(p, offset, 37);
    return (v >> 36) ? (int64_t)v - (1LL << 37) : (int64_t)v;
}

static void fix2(dronecan_t *d, uint32_t from, const uint8_t *data, uint8_t len)
{
    if (len < 2U) return;
    uint8_t tail = data[len - 1U], n = (uint8_t)(len - 1U);
    if (tail & SOT) {
        /* Fix2 never fits one frame. */
        d->receiving = !(tail & (EOT | TOGGLE)); d->source = (uint8_t)from; d->tid = tail & 31U; d->toggle = 0; d->len = 0;
    } else if (!d->receiving || from != d->source || (tail & 31U) != d->tid || (tail & TOGGLE) != d->toggle) return;
    if (!d->receiving) return;
    if (d->len + n > sizeof(d->buf)) { d->receiving = false; d->bad++; return; }
    memcpy(d->buf + d->len, data, n);
    d->len = (uint16_t)(d->len + n); d->toggle ^= TOGGLE;
    if (!(tail & EOT)) return;
    d->receiving = false;
    const uint8_t *p = d->buf + 2;
    if (d->len < 2U + 46U || transfer_crc(FIX2_SIGNATURE, p, d->len - 2U) != (uint16_t)(d->buf[0] | (d->buf[1] << 8))) { d->bad++; return; }
    int64_t lon = signed37(p, 136), lat = signed37(p, 173);
    if (lat < -9000000000LL || lat > 9000000000LL || lon < -18000000000LL || lon > 18000000000LL) { d->bad++; return; }
    d->lon_1e8 = lon; d->lat_1e8 = lat;
    d->sats = (uint8_t)bits(p, 360, 6); d->status = (uint8_t)bits(p, 366, 2);
    d->gps_node = (uint8_t)from; d->fixes++; d->fresh = true;
}

void dronecan_rx(dronecan_t *d, uint32_t id, const uint8_t *data, uint8_t len, uint32_t now_ms)
{
    if (is_allocation(id)) allocation(d, data, len, now_ms);
    else if (is_fix2(id)) fix2(d, source(id), data, len);
}

int dronecan_gga(const dronecan_t *d, char *out, unsigned size)
{
    uint64_t lat = (uint64_t)(d->lat_1e8 < 0 ? -d->lat_1e8 : d->lat_1e8), lon = (uint64_t)(d->lon_1e8 < 0 ? -d->lon_1e8 : d->lon_1e8);
    unsigned long lat_min = (unsigned long)(lat % 100000000ULL * 60ULL / 1000ULL), lon_min = (unsigned long)(lon % 100000000ULL * 60ULL / 1000ULL);
    int n = snprintf(out, size, "$GNGGA,,%02u%02lu.%05lu,%c,%03u%02lu.%05lu,%c,%d,%02u,,,M,,M,,",
                     (unsigned)(lat / 100000000ULL), lat_min / 100000UL, lat_min % 100000UL, d->lat_1e8 < 0 ? 'S' : 'N',
                     (unsigned)(lon / 100000000ULL), lon_min / 100000UL, lon_min % 100000UL, d->lon_1e8 < 0 ? 'W' : 'E',
                     d->status >= 2U, (unsigned)d->sats);
    uint8_t sum = 0;
    for (int i = 1; i < n; i++) sum ^= (uint8_t)out[i];
    return n + snprintf(out + n, size - (unsigned)n, "*%02X\r\n", sum);
}
