/* Host-test stand-in for Core/Inc/main.h: just what Modules/PiLink/uart_protocol.c needs. */
#pragma once
#include <stdint.h>
typedef struct { int unused; } UART_HandleTypeDef;
typedef enum { HAL_OK = 0, HAL_ERROR } HAL_StatusTypeDef;
static inline HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *d, uint16_t n, uint32_t t)
{ (void)h; (void)d; (void)n; (void)t; return HAL_OK; }
