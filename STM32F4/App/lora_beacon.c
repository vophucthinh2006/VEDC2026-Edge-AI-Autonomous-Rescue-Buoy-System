#include "lora_beacon.h"
#include "app_config.h"
#include "main.h"
#include "sx127x.h"
#include <stdio.h>
#include <string.h>

/* Frame (one line of text, under 100 characters because the station truncates the raw message at 100):
 *   id=PHAO-01,10.762622,106.660172,hdop=0.9,seq=12     with a fix
 *   id=PHAO-01,NO_FIX,seq=13                            no fix, or the GPS is silent
 * The station takes the first two decimal numbers without an '=' as lat/lon, and the id= field. */

#define TX_TIMEOUT_MS      1500U   /* SF9 BW125 takes ~0.4 s on air, over 1.5 s means a fault */
#define TX_POLL_PERIOD_MS  5U      /* do not poll TX_DONE every loop pass, it costs SPI time */
#define FIRST_TX_DELAY_MS  1000U

volatile lora_debug_t lora_debug;

#if LORA_ENABLED
static sx127x_t radio;
static const gps_state_t *gps_src;
static uint32_t next_tx_ms;
static uint32_t tx_start_ms;
static uint32_t last_poll_ms;

static uint16_t build_frame(char *buf, size_t size, uint32_t now_ms)
{
    bool fix = gps_src->valid && (uint32_t)(now_ms - gps_src->last_rx_ms) <= LORA_GPS_STALE_MS;
    int n;
    if (fix) {
        n = snprintf(buf, size, "id=%s,%.6f,%.6f,hdop=%.1f,seq=%lu", LORA_BUOY_ID,
                     (double)gps_src->latitude_deg, (double)gps_src->longitude_deg, (double)gps_src->hdop,
                     (unsigned long)lora_debug.seq);
    } else {
        n = snprintf(buf, size, "id=%s,NO_FIX,seq=%lu", LORA_BUOY_ID, (unsigned long)lora_debug.seq);
    }
    lora_debug.gps_valid = fix ? 1U : 0U;
    if (n < 0) return 0U;
    return (uint16_t)((size_t)n >= size ? size - 1U : (size_t)n);
}
#endif /* LORA_ENABLED */

void LoraBeacon_Init(SPI_HandleTypeDef *hspi, const gps_state_t *gps)
{
    memset((void *)&lora_debug, 0, sizeof(lora_debug));
#if !LORA_ENABLED
    lora_debug.state = LORA_STATE_DISABLED;
    (void)hspi;
    (void)gps;
#else
    gps_src = gps;
    radio.hspi = hspi;
    radio.nss_port = LORA_NSS_GPIO_Port;
    radio.nss_pin = LORA_NSS_Pin;
    radio.rst_port = LORA_RST_GPIO_Port;
    radio.rst_pin = LORA_RST_Pin;

    const sx127x_config_t cfg = {
        .frequency_hz = LORA_FREQUENCY_HZ,
        .spreading_factor = LORA_SF,
        .bandwidth_hz = LORA_BW_HZ,
        .coding_rate = LORA_CR,
        .sync_word = LORA_SYNC_WORD,
        .tx_power_dbm = LORA_TX_POWER_DBM,
        .payload_crc = true,
    };
    uint8_t version = 0U;
    sx127x_status_t st = SX127x_Init(&radio, &cfg, &version);
    lora_debug.version = version;
    lora_debug.last_error = (uint8_t)st;
    if (st != SX127X_OK) {
        lora_debug.state = LORA_STATE_INIT_FAIL;
        return;
    }
    lora_debug.init_ok = 1U;
    lora_debug.state = LORA_STATE_IDLE;
    next_tx_ms = HAL_GetTick() + FIRST_TX_DELAY_MS;
#endif
}

void LoraBeacon_Tick(uint32_t now_ms)
{
#if LORA_ENABLED
    if (lora_debug.state == LORA_STATE_IDLE) {
        if ((int32_t)(now_ms - next_tx_ms) < 0) return;

        char frame[SX127X_MAX_PAYLOAD];
        uint16_t len = build_frame(frame, sizeof(frame), now_ms);
        next_tx_ms = now_ms + LORA_BEACON_PERIOD_MS;
        lora_debug.seq++;
        if (len > 0U && SX127x_StartTx(&radio, (const uint8_t *)frame, (uint8_t)len)) {
            lora_debug.last_len = len;
            tx_start_ms = now_ms;
            last_poll_ms = now_ms;
            lora_debug.state = LORA_STATE_TX;
        } else {
            SX127x_Standby(&radio);
            lora_debug.tx_fail++;
        }
    } else if (lora_debug.state == LORA_STATE_TX) {
        if ((uint32_t)(now_ms - last_poll_ms) < TX_POLL_PERIOD_MS) return;
        last_poll_ms = now_ms;
        if (SX127x_TxDone(&radio)) {
            lora_debug.tx_count++;
            lora_debug.last_tx_ms = now_ms;
            lora_debug.state = LORA_STATE_IDLE;
        } else if ((uint32_t)(now_ms - tx_start_ms) > TX_TIMEOUT_MS) {
            SX127x_Standby(&radio);
            lora_debug.tx_fail++;
            lora_debug.state = LORA_STATE_IDLE;
        }
    }
#else
    (void)now_ms;
#endif
}
