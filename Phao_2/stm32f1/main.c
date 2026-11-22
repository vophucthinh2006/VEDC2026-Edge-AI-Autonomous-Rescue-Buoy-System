/* Stationary USB-UART / RA-02 / GPS bridge. No motor, relay or servo code. */
#include "stm32f1xx_hal.h"
#include "sx127x.h"
#include <string.h>
#include <stdio.h>

static UART_HandleTypeDef pi, gps;
static SPI_HandleTypeDef spi;
static sx127x_t radio;
static uint8_t pi_byte, gps_byte;
typedef struct { volatile uint16_t head, tail; volatile uint32_t lost; uint8_t data[1024]; } ring_t;
static ring_t pi_ring, gps_ring;
static void put(ring_t *r, uint8_t b) {
    uint16_t next = (r->head + 1U) % sizeof(r->data);
    if (next == r->tail) { r->lost++; return; }
    r->data[r->head] = b; r->head = next;
}
static int get(ring_t *r) {
    if (r->head == r->tail) return -1;
    int b = r->data[r->tail]; r->tail = (r->tail + 1U) % sizeof(r->data); return b;
}
static void say(const char *s) { HAL_UART_Transmit(&pi, (uint8_t *)s, strlen(s), 100); }
void SysTick_Handler(void) { HAL_IncTick(); }
void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&pi); }
void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&gps); }
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h) {
    if (h == &pi) { put(&pi_ring, pi_byte); HAL_UART_Receive_IT(h, &pi_byte, 1); }
    if (h == &gps) { put(&gps_ring, gps_byte); HAL_UART_Receive_IT(h, &gps_byte, 1); }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h) {
    __HAL_UART_CLEAR_OREFLAG(h);
    HAL_UART_Receive_IT(h, h == &pi ? &pi_byte : &gps_byte, 1);
}
static void fail(void) { __disable_irq(); while (1) {} }
static void clock_init(void) {
    /* Internal HSI: works without assuming a particular Blue Pill crystal. 64 MHz. */
    RCC_OscInitTypeDef o = {0}; RCC_ClkInitTypeDef c = {0};
    o.OscillatorType = RCC_OSCILLATORTYPE_HSI; o.HSIState = RCC_HSI_ON;
    o.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    o.PLL.PLLState = RCC_PLL_ON; o.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2; o.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&o) != HAL_OK) fail();
    c.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; c.AHBCLKDivider = RCC_SYSCLK_DIV1;
    c.APB1CLKDivider = RCC_HCLK_DIV2; c.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&c, FLASH_LATENCY_2) != HAL_OK) fail();
}
static void pin(GPIO_TypeDef *port, uint16_t pins, uint32_t mode) {
    GPIO_InitTypeDef g = {0}; g.Pin = pins; g.Mode = mode; g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Pull = GPIO_NOPULL; HAL_GPIO_Init(port, &g);
}
static void uart_init(UART_HandleTypeDef *h, USART_TypeDef *instance, uint32_t baud) {
    h->Instance = instance; h->Init.BaudRate = baud; h->Init.WordLength = UART_WORDLENGTH_8B;
    h->Init.StopBits = UART_STOPBITS_1; h->Init.Parity = UART_PARITY_NONE;
    h->Init.Mode = UART_MODE_TX_RX; h->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    h->Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(h) != HAL_OK) fail();
}
int main(void) {
    HAL_Init(); clock_init();
    __HAL_RCC_AFIO_CLK_ENABLE(); __HAL_AFIO_REMAP_SWJ_NOJTAG();
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE(); __HAL_RCC_USART2_CLK_ENABLE(); __HAL_RCC_SPI1_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    pin(GPIOA, GPIO_PIN_4, GPIO_MODE_OUTPUT_PP); pin(GPIOB, GPIO_PIN_0, GPIO_MODE_OUTPUT_PP);
    pin(GPIOA, GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_2, GPIO_MODE_AF_PP);
    pin(GPIOA, GPIO_PIN_6|GPIO_PIN_10|GPIO_PIN_3, GPIO_MODE_INPUT);
    uart_init(&pi, USART1, 115200); uart_init(&gps, USART2, GPS_BAUD);
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0); HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0); HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_UART_Receive_IT(&pi, &pi_byte, 1); HAL_UART_Receive_IT(&gps, &gps_byte, 1);
    spi.Instance = SPI1; spi.Init.Mode = SPI_MODE_MASTER; spi.Init.Direction = SPI_DIRECTION_2LINES;
    spi.Init.DataSize = SPI_DATASIZE_8BIT; spi.Init.CLKPolarity = SPI_POLARITY_LOW;
    spi.Init.CLKPhase = SPI_PHASE_1EDGE; spi.Init.NSS = SPI_NSS_SOFT;
    spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spi.Init.TIMode = SPI_TIMODE_DISABLE; spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    if (HAL_SPI_Init(&spi) != HAL_OK) fail();
    radio = (sx127x_t){ &spi, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0 };
    const sx127x_config_t cfg = {433000000, 9, 125000, 5, 0xF3, 17, true};
    uint8_t version;
    bool ready = SX127x_Init(&radio, &cfg, &version) == SX127X_OK;
    if (ready) SX127x_StartRx(&radio);
    say(ready ? "READY,2\n" : "ERROR,RADIO_INIT\n");
    char line[132]; unsigned used = 0; bool overflow = false, transmitting = false;
    uint32_t started = 0, last_gps = 0, last_status = 0, last_radio = 0;
    while (1) {
        uint32_t now = HAL_GetTick(); int b;
        /* A power dip resets the RA-02 to FSK defaults (OpMode 0x04, 434 MHz) while the F103
           keeps running: no RX, and its flag register fakes TX_DONE. Re-init when LoRa bit is gone. */
        if (!transmitting && (uint32_t)(now-last_radio) >= 1000) {
            last_radio = now;
            uint8_t op = SX127x_ReadReg(&radio, 0x01U);   /* RegOpMode, bit 7 = LoRa */
            if (!ready || op == 0xFFU || (op & 0x80U) == 0U) {
                bool was_ready = ready;
                ready = SX127x_Init(&radio, &cfg, &version) == SX127X_OK;
                if (ready) SX127x_StartRx(&radio);
                if (was_ready || ready) say(ready ? "ERROR,RADIO_RESET\n" : "ERROR,RADIO_INIT\n");
            }
        }
        while ((b = get(&pi_ring)) >= 0) {
            if (b == '\r') continue;
            if (b == '\n') {
                line[used] = 0;
                if (!overflow && !strcmp(line, "PING")) say(ready ? "READY,2\n" : "ERROR,RADIO_INIT\n");
                else if (!overflow && !strncmp(line, "TX,", 3) && used > 3 && used <= 123) {
                    if (!ready) say("ERROR,RADIO_INIT\n");
                    else if (transmitting) say("ERROR,BUSY\n");
                    else if (SX127x_StartTx(&radio, (uint8_t *)line+3, used-3)) {
                        transmitting = true; started = now; say("TXSTART\n");
                    } else { say("ERROR,TX\n"); SX127x_StartRx(&radio); }
                } else say("ERROR,COMMAND\n");
                used = 0; overflow = false;
            } else if (used < sizeof(line)-1) line[used++] = (char)b;
            else overflow = true;
        }
        if (transmitting && (SX127x_TxDone(&radio) || (uint32_t)(now-started) > 2000)) {
            say((uint32_t)(now-started) > 2000 ? "ERROR,TX_TIMEOUT\n" : "TXDONE\n");
            transmitting = false; SX127x_StartRx(&radio);
        }
        if (ready && !transmitting) {
            uint8_t packet[121], n = 0;
            if (SX127x_ReadPacket(&radio, packet, 120, &n)) {
                /* Radio protocol is ASCII; reject embedded line/control characters. */
                bool ascii = true;
                for (unsigned i=0; i<n; i++) if (packet[i]<32 || packet[i]>126) ascii=false;
                if (ascii) { packet[n]=0; say("RX,"); say((char *)packet); say("\n"); }
            }
        }
        if ((uint32_t)(now-last_gps) >= 10) {
            last_gps = now;
            /* Up to 4 lines of 64 B per pass (25.6 kB/s): a single line (6.4 kB/s) fell behind a
               continuous 115200 GPS (11.5 kB/s). Capped because say() blocks: hex doubles the
               bytes, so a full 1 kB ring would stall the radio loop ~180 ms on the 115200 Pi link. */
            bool more = true;
            for (unsigned line_no = 0; more && line_no < 4; line_no++) {
                char out[138] = "GPSHEX,"; unsigned n = 7;
                for (unsigned i=0; i<64 && (more = (b=get(&gps_ring))>=0); i++) {
                    static const char hex[]="0123456789ABCDEF";
                    out[n++]=hex[b>>4]; out[n++]=hex[b&15];
                }
                if (n>7) { out[n++]='\n'; out[n]=0; say(out); }
            }
        }
        if ((uint32_t)(now-last_status) >= 5000) {
            char out[90]; last_status=now;
            snprintf(out,sizeof(out),"STATUS,pi_lost=%lu,gps_lost=%lu\n",(unsigned long)pi_ring.lost,(unsigned long)gps_ring.lost);
            say(out);
        }
    }
}
