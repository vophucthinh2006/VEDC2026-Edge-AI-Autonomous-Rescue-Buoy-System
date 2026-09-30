#pragma once

/* SX1276/77/78/79 driver (RA-02 module) for the STM32 HAL, transmit only.
 * It knows nothing about the App layer: settings come in through a struct, pins through a handle.
 * Non-blocking: SX127x_StartTx() only loads the FIFO and starts TX, SX127x_TxDone() checks the flag.
 * Registers follow the Semtech datasheet, with the same formulas as
 * esp32-lora-station/components/sx127x so both ends end up with the same radio settings. */

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

#define SX127X_VERSION_EXPECTED  0x12U
#define SX127X_MAX_PAYLOAD       100U

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *nss_port;
    uint16_t nss_pin;
    GPIO_TypeDef *rst_port;
    uint16_t rst_pin;
} sx127x_t;

typedef struct {
    uint32_t frequency_hz;      /* e.g. 433000000 */
    uint8_t spreading_factor;   /* 6..12 */
    uint32_t bandwidth_hz;      /* e.g. 125000 */
    uint8_t coding_rate;        /* denominator of 4/x, 5..8 */
    uint8_t sync_word;          /* must match the receiver */
    uint8_t tx_power_dbm;       /* 2..20, PA_BOOST */
    bool payload_crc;           /* append a CRC (the receiver checks it from the header flag) */
} sx127x_config_t;

typedef enum {
    SX127X_OK = 0,
    SX127X_ERR_SPI,             /* HAL_SPI returned an error */
    SX127X_ERR_CHIP,            /* REG_VERSION is not 0x12: wrong wiring, no power, or no module */
    SX127X_ERR_ARG              /* parameter out of range */
} sx127x_status_t;

/* Reset through the RST pin, read REG_VERSION, configure the radio, leave it in standby.
 * Blocks for about 70 ms (HAL_Delay): call it only at start-up, before the control loop.
 * version_out (may be NULL) receives the REG_VERSION value read, even on error. */
sx127x_status_t SX127x_Init(sx127x_t *dev, const sx127x_config_t *cfg, uint8_t *version_out);

/* Load the FIFO, start TX and return at once. False if len is too big or SPI fails. */
bool SX127x_StartTx(sx127x_t *dev, const uint8_t *data, uint8_t len);

/* True once the packet is out (clears the flag, back to standby). Poll it after StartTx. */
bool SX127x_TxDone(sx127x_t *dev);

/* Abort TX, back to standby. */
void SX127x_Standby(sx127x_t *dev);

/* Read one register (diagnostics). */
uint8_t SX127x_ReadReg(sx127x_t *dev, uint8_t addr);
