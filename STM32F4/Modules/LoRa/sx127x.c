#include "sx127x.h"

#define REG_FIFO                 0x00U
#define REG_OP_MODE              0x01U
#define REG_FRF_MSB              0x06U
#define REG_FRF_MID              0x07U
#define REG_FRF_LSB              0x08U
#define REG_PA_CONFIG            0x09U
#define REG_LNA                  0x0CU
#define REG_FIFO_ADDR_PTR        0x0DU
#define REG_FIFO_TX_BASE_ADDR    0x0EU
#define REG_FIFO_RX_BASE_ADDR    0x0FU
#define REG_IRQ_FLAGS            0x12U
#define REG_MODEM_CONFIG_1       0x1DU
#define REG_MODEM_CONFIG_2       0x1EU
#define REG_PREAMBLE_MSB         0x20U
#define REG_PREAMBLE_LSB         0x21U
#define REG_PAYLOAD_LENGTH       0x22U
#define REG_MODEM_CONFIG_3       0x26U
#define REG_DETECTION_OPTIMIZE   0x31U
#define REG_DETECTION_THRESHOLD  0x37U
#define REG_SYNC_WORD            0x39U
#define REG_VERSION              0x42U
#define REG_PA_DAC               0x4DU

#define MODE_LONG_RANGE          0x80U   /* bit 7 = LoRa mode */
#define MODE_SLEEP               0x00U
#define MODE_STDBY               0x01U
#define MODE_TX                  0x03U

#define IRQ_TX_DONE              0x08U

#define SPI_TIMEOUT_MS           5U

static void nss(sx127x_t *d, GPIO_PinState level) { HAL_GPIO_WritePin(d->nss_port, d->nss_pin, level); }

static bool write_reg(sx127x_t *d, uint8_t addr, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(addr | 0x80U), value };
    nss(d, GPIO_PIN_RESET);
    HAL_StatusTypeDef st = HAL_SPI_Transmit(d->hspi, tx, 2U, SPI_TIMEOUT_MS);
    nss(d, GPIO_PIN_SET);
    return st == HAL_OK;
}

uint8_t SX127x_ReadReg(sx127x_t *d, uint8_t addr)
{
    uint8_t tx[2] = { (uint8_t)(addr & 0x7FU), 0U };
    uint8_t rx[2] = { 0U, 0U };
    nss(d, GPIO_PIN_RESET);
    (void)HAL_SPI_TransmitReceive(d->hspi, tx, rx, 2U, SPI_TIMEOUT_MS);
    nss(d, GPIO_PIN_SET);
    return rx[1];
}

static bool write_fifo(sx127x_t *d, const uint8_t *data, uint8_t len)
{
    uint8_t buf[SX127X_MAX_PAYLOAD + 1U];
    buf[0] = (uint8_t)(REG_FIFO | 0x80U);
    for (uint8_t i = 0U; i < len; i++) buf[1U + i] = data[i];
    nss(d, GPIO_PIN_RESET);
    HAL_StatusTypeDef st = HAL_SPI_Transmit(d->hspi, buf, (uint16_t)(len + 1U), SPI_TIMEOUT_MS);
    nss(d, GPIO_PIN_SET);
    return st == HAL_OK;
}

static bool set_mode(sx127x_t *d, uint8_t mode) { return write_reg(d, REG_OP_MODE, (uint8_t)(MODE_LONG_RANGE | mode)); }

static uint8_t bandwidth_bits(uint32_t hz)
{
    if (hz <= 7800U)   return 0U;
    if (hz <= 10400U)  return 1U;
    if (hz <= 15600U)  return 2U;
    if (hz <= 20800U)  return 3U;
    if (hz <= 31250U)  return 4U;
    if (hz <= 41700U)  return 5U;
    if (hz <= 62500U)  return 6U;
    if (hz <= 125000U) return 7U;
    if (hz <= 250000U) return 8U;
    return 9U;                                 /* 500 kHz */
}

sx127x_status_t SX127x_Init(sx127x_t *d, const sx127x_config_t *c, uint8_t *version_out)
{
    if (c->spreading_factor < 6U || c->spreading_factor > 12U || c->coding_rate < 5U || c->coding_rate > 8U ||
        c->tx_power_dbm < 2U || c->tx_power_dbm > 20U) return SX127X_ERR_ARG;

    nss(d, GPIO_PIN_SET);
    HAL_GPIO_WritePin(d->rst_port, d->rst_pin, GPIO_PIN_RESET);
    HAL_Delay(10U);
    HAL_GPIO_WritePin(d->rst_port, d->rst_pin, GPIO_PIN_SET);
    HAL_Delay(60U);                            /* some clone modules need more than 10 ms */

    uint8_t version = SX127x_ReadReg(d, REG_VERSION);
    if (version_out != NULL) *version_out = version;
    if (version != SX127X_VERSION_EXPECTED) return SX127X_ERR_CHIP;

    bool ok = set_mode(d, MODE_SLEEP);         /* the LoRa bit can only change in sleep */

    /* Frequency: FRF = f * 2^19 / 32 MHz */
    uint64_t frf = ((uint64_t)c->frequency_hz << 19) / 32000000ULL;
    ok &= write_reg(d, REG_FRF_MSB, (uint8_t)(frf >> 16));
    ok &= write_reg(d, REG_FRF_MID, (uint8_t)(frf >> 8));
    ok &= write_reg(d, REG_FRF_LSB, (uint8_t)frf);

    ok &= write_reg(d, REG_FIFO_TX_BASE_ADDR, 0x00U);
    ok &= write_reg(d, REG_FIFO_RX_BASE_ADDR, 0x00U);
    ok &= write_reg(d, REG_LNA, 0x23U);        /* LNA boost, automatic gain */

    /* BW | CR | explicit header (bit 0 = 0) */
    ok &= write_reg(d, REG_MODEM_CONFIG_1,
                    (uint8_t)((bandwidth_bits(c->bandwidth_hz) << 4) | ((c->coding_rate - 4U) << 1)));
    /* SF | payload CRC | symbol timeout MSB = 0 */
    ok &= write_reg(d, REG_MODEM_CONFIG_2,
                    (uint8_t)((c->spreading_factor << 4) | (c->payload_crc ? 0x04U : 0x00U)));
    ok &= write_reg(d, REG_MODEM_CONFIG_3, 0x04U);   /* AGC on; LowDataRateOptimize off (symbol < 16 ms) */

    if (c->spreading_factor == 6U) {           /* detection optimisation for SF6, per the datasheet */
        ok &= write_reg(d, REG_DETECTION_OPTIMIZE, 0xC5U);
        ok &= write_reg(d, REG_DETECTION_THRESHOLD, 0x0CU);
    } else {
        ok &= write_reg(d, REG_DETECTION_OPTIMIZE, 0xC3U);
        ok &= write_reg(d, REG_DETECTION_THRESHOLD, 0x0AU);
    }
    ok &= write_reg(d, REG_SYNC_WORD, c->sync_word);
    ok &= write_reg(d, REG_PREAMBLE_MSB, 0x00U);
    ok &= write_reg(d, REG_PREAMBLE_LSB, 0x08U);     /* 8 symbols, the default, matches the station */

    /* Power: the RA-02 uses the PA_BOOST pin */
    if (c->tx_power_dbm > 17U) {
        ok &= write_reg(d, REG_PA_DAC, 0x87U);
        ok &= write_reg(d, REG_PA_CONFIG, (uint8_t)(0x80U | 0x70U | (c->tx_power_dbm - 5U)));
    } else {
        ok &= write_reg(d, REG_PA_DAC, 0x84U);
        ok &= write_reg(d, REG_PA_CONFIG, (uint8_t)(0x80U | (c->tx_power_dbm - 2U)));
    }

    ok &= set_mode(d, MODE_STDBY);
    return ok ? SX127X_OK : SX127X_ERR_SPI;
}

bool SX127x_StartTx(sx127x_t *d, const uint8_t *data, uint8_t len)
{
    if (len == 0U || len > SX127X_MAX_PAYLOAD) return false;
    bool ok = set_mode(d, MODE_STDBY);
    ok &= write_reg(d, REG_FIFO_ADDR_PTR, 0x00U);
    ok &= write_fifo(d, data, len);
    ok &= write_reg(d, REG_PAYLOAD_LENGTH, len);
    ok &= write_reg(d, REG_IRQ_FLAGS, 0xFFU);        /* clear stale flags */
    ok &= set_mode(d, MODE_TX);
    return ok;
}

bool SX127x_TxDone(sx127x_t *d)
{
    if ((SX127x_ReadReg(d, REG_IRQ_FLAGS) & IRQ_TX_DONE) == 0U) return false;
    (void)write_reg(d, REG_IRQ_FLAGS, 0xFFU);
    (void)set_mode(d, MODE_STDBY);
    return true;
}

void SX127x_Standby(sx127x_t *d)
{
    (void)set_mode(d, MODE_STDBY);
}
