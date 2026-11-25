/* Stationary USB-UART / RA-02 / DroneCAN GPS bridge. No motor, relay or servo code. */
#include "stm32f1xx_hal.h"
#include "sx127x.h"
#include "dronecan.h"
#include <string.h>
#include <stdio.h>

static UART_HandleTypeDef pi;
static SPI_HandleTypeDef spi;
static sx127x_t radio;
static uint8_t pi_byte;
typedef struct { volatile uint16_t head, tail; volatile uint32_t lost; uint8_t data[1024]; } ring_t;
static ring_t pi_ring;
/* A Fix2 is a burst of about ten frames; say() can block the loop for milliseconds. */
typedef struct { uint32_t id; uint8_t len, data[8]; } frame_t;
static frame_t can_ring[64];
static volatile uint8_t can_head, can_tail;
static volatile uint32_t can_seen, can_lost;
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
void USB_LP_CAN1_RX0_IRQHandler(void) {
    uint32_t rir = CAN1->sFIFOMailBox[0].RIR, id = rir >> 3, lo = CAN1->sFIFOMailBox[0].RDLR, hi = CAN1->sFIFOMailBox[0].RDHR;
    uint8_t len = CAN1->sFIFOMailBox[0].RDTR & 15U;
    if (CAN1->RF0R & CAN_RF0R_FOVR0) can_lost++;
    CAN1->RF0R = CAN_RF0R_RFOM0 | CAN_RF0R_FOVR0;
    can_seen++;
    if (!(rir & CAN_RI0R_IDE) || (rir & CAN_RI0R_RTR) || len > 8U || !dronecan_wanted(id)) return;
    uint8_t next = (can_head + 1U) % 64U;
    if (next == can_tail) { can_lost++; return; }
    frame_t *f = &can_ring[can_head];
    f->id = id; f->len = len; memcpy(f->data, &lo, 4); memcpy(f->data+4, &hi, 4);
    can_head = next;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h) {
    if (h == &pi) { put(&pi_ring, pi_byte); HAL_UART_Receive_IT(h, &pi_byte, 1); }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h) {
    __HAL_UART_CLEAR_OREFLAG(h);
    HAL_UART_Receive_IT(h, &pi_byte, 1);
}
static void fail(void) { __disable_irq(); while (1) {} }
static void clock_init(void) {
    /* 64 MHz. The 8 MHz crystal: 1 Mbit/s CAN has little margin left at the HSI's 1 %. Without one, the HSI. */
    RCC_OscInitTypeDef o = {0}; RCC_ClkInitTypeDef c = {0};
    o.OscillatorType = RCC_OSCILLATORTYPE_HSE; o.HSEState = RCC_HSE_ON; o.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    o.PLL.PLLState = RCC_PLL_ON; o.PLL.PLLSource = RCC_PLLSOURCE_HSE; o.PLL.PLLMUL = RCC_PLL_MUL8;
    if (HAL_RCC_OscConfig(&o) != HAL_OK) {
        o.OscillatorType = RCC_OSCILLATORTYPE_HSI; o.HSIState = RCC_HSI_ON;
        o.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        o.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2; o.PLL.PLLMUL = RCC_PLL_MUL16;
        if (HAL_RCC_OscConfig(&o) != HAL_OK) fail();
    }
    c.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; c.AHBCLKDivider = RCC_SYSCLK_DIV1;
    c.APB1CLKDivider = RCC_HCLK_DIV2;
    c.APB2CLKDivider = RCC_HCLK_DIV8;   /* 8 MHz: the only way to get SPI1 (max. prescaler 256) down to 31 kHz */
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
/* bxCAN at 1 Mbit/s on PA11 (RX) / PA12 (TX): APB1 32 MHz / 2 / (1+12+3), SJW 3. */
static bool can_wait(uint32_t inak) {
    uint32_t start = HAL_GetTick();
    while ((CAN1->MSR & CAN_MSR_INAK) != inak) if (HAL_GetTick()-start > 20) return false;
    return true;
}
static bool can_init(void) {
    GPIO_InitTypeDef g = { .Pin = GPIO_PIN_11, .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLUP };
    HAL_GPIO_Init(GPIOA, &g); pin(GPIOA, GPIO_PIN_12, GPIO_MODE_AF_PP);
    __HAL_RCC_CAN1_CLK_ENABLE();
    CAN1->MCR = CAN_MCR_INRQ | CAN_MCR_ABOM | CAN_MCR_TXFP;
    if (!can_wait(CAN_MSR_INAK)) return false;
    CAN1->BTR = (2U << CAN_BTR_SJW_Pos) | (2U << CAN_BTR_TS2_Pos) | (11U << CAN_BTR_TS1_Pos) | 1U;
    CAN1->FMR |= CAN_FMR_FINIT;
    CAN1->FS1R |= 1U; CAN1->FM1R &= ~1U; CAN1->FFA1R &= ~1U;
    CAN1->sFilterRegister[0].FR1 = 0; CAN1->sFilterRegister[0].FR2 = 0;
    CAN1->FA1R |= 1U;
    CAN1->FMR &= ~CAN_FMR_FINIT;
    CAN1->IER = CAN_IER_FMPIE0;
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 1, 0); HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    CAN1->MCR &= ~CAN_MCR_INRQ;
    return can_wait(0);
}
static bool can_send(uint32_t id, const uint8_t *data, uint8_t len) {
    uint32_t start = HAL_GetTick(), lo = 0, hi = 0;
    /* Three mailboxes; nobody on the bus leaves them full, so do not wait long. */
    while (!(CAN1->TSR & CAN_TSR_TME)) if (HAL_GetTick()-start > 2) return false;
    uint32_t box = (CAN1->TSR & CAN_TSR_TME0) ? 0 : (CAN1->TSR & CAN_TSR_TME1) ? 1 : 2;
    memcpy(&lo, data, len > 4 ? 4 : len);
    if (len > 4) memcpy(&hi, data+4, len-4U);
    CAN1->sTxMailBox[box].TDTR = len; CAN1->sTxMailBox[box].TDLR = lo; CAN1->sTxMailBox[box].TDHR = hi;
    CAN1->sTxMailBox[box].TIR = (id << 3) | CAN_TI0R_IDE | CAN_TI0R_TXRQ;
    return true;
}
int main(void) {
    HAL_Init(); clock_init();
    __HAL_RCC_AFIO_CLK_ENABLE(); __HAL_AFIO_REMAP_SWJ_NOJTAG();
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE(); __HAL_RCC_SPI1_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    pin(GPIOB, GPIO_PIN_0, GPIO_MODE_OUTPUT_PP);
    pin(GPIOA, GPIO_PIN_9, GPIO_MODE_AF_PP);
    GPIO_InitTypeDef bus = { .Pin = GPIO_PIN_5|GPIO_PIN_7, .Mode = GPIO_MODE_AF_PP, .Speed = GPIO_SPEED_FREQ_LOW };
    HAL_GPIO_Init(GPIOA, &bus);   /* slow edges: see the SPI clock below */
    bus.Pin = GPIO_PIN_4; bus.Mode = GPIO_MODE_OUTPUT_PP; HAL_GPIO_Init(GPIOA, &bus);
    pin(GPIOA, GPIO_PIN_6, GPIO_MODE_INPUT);
    /* Pull-up: unplugged, a floating PA10 picks up the CAN pin beside it as a flood of bytes. */
    GPIO_InitTypeDef rx = { .Pin = GPIO_PIN_10, .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLUP };
    HAL_GPIO_Init(GPIOA, &rx);
    uart_init(&pi, USART1, 115200);
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0); HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_UART_Receive_IT(&pi, &pi_byte, 1);
    static dronecan_t can = { .send = can_send };
    bool can_ok = can_init();
    spi.Instance = SPI1; spi.Init.Mode = SPI_MODE_MASTER; spi.Init.Direction = SPI_DIRECTION_2LINES;
    spi.Init.DataSize = SPI_DATASIZE_8BIT; spi.Init.CLKPolarity = SPI_POLARITY_LOW;
    spi.Init.CLKPhase = SPI_PHASE_1EDGE; spi.Init.NSS = SPI_NSS_SOFT;
    /* 31 kHz with slow edges. Measured on this wiring: at 125 kHz and up RA-02 loses 1 bits on MOSI (writes
       become reads, RADIO_RESET every second), 62 kHz and below with slow edges has no errors. */
    spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spi.Init.TIMode = SPI_TIMODE_DISABLE; spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    if (HAL_SPI_Init(&spi) != HAL_OK) fail();
    radio = (sx127x_t){ &spi, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0 };
    const sx127x_config_t cfg = {433000000, 9, 125000, 5, 0xF3, 17, true};
    uint8_t version;
    bool ready = SX127x_Init(&radio, &cfg, &version) == SX127X_OK;
    if (ready) SX127x_StartRx(&radio);
    say(ready ? "READY,2\n" : "ERROR,RADIO_INIT\n");
    char line[132]; unsigned used = 0; bool overflow = false, transmitting = false;
    uint32_t started = 0, last_gps = 0, last_node = 0, last_status = 0, last_radio = 0;
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
        while (can_tail != can_head) {
            frame_t *f = &can_ring[can_tail];
            dronecan_rx(&can, f->id, f->data, f->len, now);
            can_tail = (can_tail + 1U) % 64U;
        }
        if (can_ok && (uint32_t)(now-last_node) >= 1000) { last_node = now; dronecan_status(&can, now/1000U); }
        if (can.fresh && (uint32_t)(now-last_gps) >= 1000) {
            /* To the Pi as NMEA, in the same GPSHEX lines as before. */
            static const char hex[]="0123456789ABCDEF";
            char gga[96], out[210] = "GPSHEX,"; unsigned n = 7;
            last_gps = now; can.fresh = false;
            int length = dronecan_gga(&can, gga, sizeof(gga));
            for (int i = 0; i < length; i++) { out[n++]=hex[(gga[i]>>4)&15]; out[n++]=hex[gga[i]&15]; }
            out[n++]='\n'; out[n]=0; say(out);
        }
        if ((uint32_t)(now-last_status) >= 5000) {
            char out[200]; last_status=now;
            /* can_esr: bxCAN ESR, REC in the top byte, TEC below it, bit 2 bus-off, bits 6..4 last error. */
            snprintf(out,sizeof(out),"STATUS,pi_lost=%lu,can=%d,can_rx=%lu,can_lost=%lu,can_esr=%08lx,gps_node=%u,gps_fix=%u,gps_sats=%u,gps_msgs=%lu,gps_bad=%lu\n",
                     (unsigned long)pi_ring.lost,can_ok,(unsigned long)can_seen,(unsigned long)can_lost,(unsigned long)CAN1->ESR,
                     can.gps_node,can.status,can.sats,(unsigned long)can.fixes,(unsigned long)can.bad);
            say(out);
        }
    }
}
