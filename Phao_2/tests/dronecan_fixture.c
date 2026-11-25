/* stdin: "ms id data" (hex id and data) per CAN frame. stdout: frames sent, then the GGA after each new fix. */
#include "dronecan.h"
#include <stdio.h>
#include <string.h>

static bool send(uint32_t id, const uint8_t *data, uint8_t len)
{
    printf("TX %08x ", (unsigned)id);
    for (int i = 0; i < len; i++) printf("%02x", data[i]);
    printf("\n");
    return true;
}

int main(void)
{
    dronecan_t d = { .send = send };
    unsigned ms, id; char hex[20], out[96];
    while (scanf("%u %x %19s", &ms, &id, hex) == 3) {
        uint8_t data[8]; unsigned n = (unsigned)strlen(hex) / 2U, b;
        for (unsigned i = 0; i < n && i < 8U; i++) { if (sscanf(hex + 2 * i, "%2x", &b) != 1) return 1; data[i] = (uint8_t)b; }
        if (dronecan_wanted(id)) dronecan_rx(&d, id, data, (uint8_t)n, ms);
        if (d.fresh) { d.fresh = false; dronecan_gga(&d, out, sizeof(out)); fputs(out, stdout); }
    }
    printf("END fixes=%u bad=%u status=%u sats=%u node=%u\n", (unsigned)d.fixes, (unsigned)d.bad, d.status, d.sats, d.gps_node);
    return 0;
}
