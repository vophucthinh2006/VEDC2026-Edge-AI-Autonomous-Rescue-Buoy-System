#include "app.h"
#include "app_config.h"
#include "i2c.h"
#include "ibus_sensor.h"
#include "ina226.h"
#include "lora_frame.h"
#include "pi_link.h"
#include "spi.h"
#include "sx127x.h"
#include "tim.h"
#include "usart.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define FAST_PERIOD_MS       10U
#define LORA_TX_TIMEOUT_MS   1500U   /* SF9 BW125 takes ~0.4 s on air, over 1.5 s means a fault */
#define LORA_POLL_PERIOD_MS  5U
#define LORA_FIRST_TX_MS     1000U

enum { LORA_OFF = 0, LORA_INIT_FAIL = 1, LORA_IDLE = 2, LORA_TX = 3 };

volatile helper_debug_t helper_debug;

static pi_link_t pi;                 /* written by the USART1 interrupt */
static uint8_t pi_rx_byte;
static char pi_tx[64];               /* stays valid while the interrupt sends it */

static ibus_sensor_t sensor;         /* fed by the USART3 interrupt */
static uint8_t ibus_rx_byte;
static volatile uint8_t ibus_reply_length;
static uint8_t ibus_reply[IBUS_SENSOR_REPLY_MAX];

static ina226_t ina = { &hi2c1, INA226_ADDRESS, INA226_CALIBRATION, INA226_CURRENT_LSB_UA };
static sx127x_t radio;

static uint32_t last_fast_ms, last_ina_ms, last_report_ms, alarm_until_ms;
static uint32_t lora_next_ms, lora_start_ms, lora_poll_ms;
static uint16_t report_sequence;
static uint8_t seen_victims, estop_ticks;
static bool alarm_on;

/* Signed: an interrupt can stamp a tick newer than the now_ms the loop read. */
static bool within(uint32_t now_ms, uint32_t stamp_ms, uint32_t timeout_ms) {
    return (int32_t)(now_ms - stamp_ms) <= (int32_t)timeout_ms;
}

static void relays(bool on) {
    GPIO_PinState level = on ? RELAY_ON_LEVEL : (RELAY_ON_LEVEL == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, RELAY1_Pin | RELAY2_Pin, level);
}

static void lora_init(void) {
#if LORA_ENABLED
    radio.hspi = &hspi1;
    radio.nss_port = LORA_NSS_GPIO_Port;
    radio.nss_pin = LORA_NSS_Pin;
    radio.rst_port = LORA_RST_GPIO_Port;
    radio.rst_pin = LORA_RST_Pin;
    const sx127x_config_t cfg = {
        .frequency_hz = LORA_FREQUENCY_HZ, .spreading_factor = LORA_SF, .bandwidth_hz = LORA_BW_HZ, .coding_rate = LORA_CR,
        .sync_word = LORA_SYNC_WORD, .tx_power_dbm = LORA_TX_POWER_DBM, .payload_crc = true,
    };
    uint8_t version = 0U;
    sx127x_status_t status = SX127x_Init(&radio, &cfg, &version);
    helper_debug.lora_version = version;
    helper_debug.lora_state = status == SX127X_OK ? LORA_IDLE : LORA_INIT_FAIL;
    lora_next_ms = HAL_GetTick() + LORA_FIRST_TX_MS;
#else
    helper_debug.lora_state = LORA_OFF;
#endif
}

static void lora_tick(uint32_t now_ms) {
#if LORA_ENABLED
    if (helper_debug.lora_state == LORA_IDLE) {
        if ((int32_t)(now_ms - lora_next_ms) < 0) return;
        /* The Pi link's interrupt rewrites the status text: copy it with that interrupt held off. */
        char status[PI_LINK_STATUS_MAX];
        bool fresh;
        HAL_NVIC_DisableIRQ(USART1_IRQn);
        fresh = pi.status_seen && within(now_ms, pi.last_status_ms, PI_STATUS_STALE_MS);
        memcpy(status, pi.status, sizeof(status));
        HAL_NVIC_EnableIRQ(USART1_IRQn);

        char frame[LORA_FRAME_SIZE];
        size_t length = LoraFrame_Build(frame, sizeof(frame), LORA_BUOY_ID, fresh ? status : NULL, helper_debug.lora_seq, seen_victims);
        lora_next_ms = now_ms + LORA_BEACON_PERIOD_MS;
        helper_debug.lora_seq++;
        if (length > 0U && SX127x_StartTx(&radio, (const uint8_t *)frame, (uint8_t)length)) {
            lora_start_ms = lora_poll_ms = now_ms;
            helper_debug.lora_state = LORA_TX;
        } else {
            SX127x_Standby(&radio);
            helper_debug.lora_fail++;
        }
    } else if (helper_debug.lora_state == LORA_TX) {
        if ((uint32_t)(now_ms - lora_poll_ms) < LORA_POLL_PERIOD_MS) return;   /* polling costs SPI time */
        lora_poll_ms = now_ms;
        if (SX127x_TxDone(&radio)) {
            helper_debug.lora_tx++;
            helper_debug.lora_state = LORA_IDLE;
        } else if ((uint32_t)(now_ms - lora_start_ms) > LORA_TX_TIMEOUT_MS) {
            SX127x_Standby(&radio);
            helper_debug.lora_fail++;
            helper_debug.lora_state = LORA_IDLE;
        }
    }
#else
    (void)now_ms;
#endif
}

/* The receiver waits only a few ms for the answer: sent from the loop, ahead of everything else. */
static void ibus_answer(void) {
    uint8_t length = ibus_reply_length;
    if (length == 0U) return;
    HAL_HalfDuplex_EnableTransmitter(&huart3);
    (void)HAL_UART_Transmit(&huart3, ibus_reply, length, 5U);
    HAL_HalfDuplex_EnableReceiver(&huart3);
    ibus_reply_length = 0U;
    helper_debug.ibus_polls = sensor.polls;
}

static void fast_tick(uint32_t now_ms) {
    bool pi_ok = pi.seen && within(now_ms, pi.last_hbt_ms, PI_LINK_TIMEOUT_MS);
    helper_debug.pi_ok = pi_ok ? 1U : 0U;
    helper_debug.pi_rx_ok = pi.rx_ok;
    helper_debug.pi_rx_bad = pi.rx_bad;

    /* Camera servo: the Pi's angle while it keeps coming, straight ahead otherwise. */
    int32_t pan = pi.cam_seen && within(now_ms, pi.last_cam_ms, CAM_TIMEOUT_MS) ? pi.cam_pan_ddeg : 0;
    if (pan > CAM_PAN_LIMIT_DDEG) pan = CAM_PAN_LIMIT_DDEG;
    if (pan < -CAM_PAN_LIMIT_DDEG) pan = -CAM_PAN_LIMIT_DDEG;
    uint32_t pulse = (uint32_t)(CAM_CENTER_US + CAM_PAN_DIR * pan * CAM_US_PER_10DEG / 100);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse);
    helper_debug.servo_us = (uint16_t)pulse;

    /* Alarm: a higher count than the last one seen. A lower one is the Pi starting again. */
    uint8_t victims = pi.victims;
    if (victims > seen_victims) {
        alarm_until_ms = now_ms + ALARM_HOLD_MS;
        alarm_on = true;
        lora_next_ms = now_ms;            /* the shore hears of it at once, not at the next period */
    }
    seen_victims = victims;
    if (alarm_on && (int32_t)(now_ms - alarm_until_ms) >= 0) alarm_on = false;
    relays(alarm_on);
    helper_debug.victims = victims;
    helper_debug.alarm = alarm_on ? 1U : 0U;

    /* E-stop button, debounced over ESTOP_DEBOUNCE_TICKS samples. */
    bool pressed = HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == ESTOP_PRESSED_LEVEL;
    if (!pressed) estop_ticks = 0U;
    else if (estop_ticks < ESTOP_DEBOUNCE_TICKS) estop_ticks++;
    helper_debug.estop = estop_ticks >= ESTOP_DEBOUNCE_TICKS ? 1U : 0U;

    /* LED (lit when low): one blink a second with the Pi talking, five without. */
    uint32_t half = pi_ok ? 500U : 100U;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, (now_ms / half) % 2U == 0U ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void ina_tick(uint32_t now_ms) {
#if INA226_ENABLED
    uint32_t period = helper_debug.ina_ok ? INA226_PERIOD_MS : INA226_RETRY_MS;
    if ((uint32_t)(now_ms - last_ina_ms) < period) return;
    last_ina_ms = now_ms;
    if (!helper_debug.ina_ok) {
        helper_debug.ina_ok = INA226_Init(&ina) ? 1U : 0U;
        return;
    }
    uint16_t bus_mv;
    int32_t current_ma;
    if (INA226_Read(&ina, &bus_mv, &current_ma)) {
        helper_debug.bus_mv = bus_mv;
        helper_debug.current_ma = current_ma;
        sensor.value = (uint16_t)(bus_mv / 10U);      /* the handset shows 0.01 V per count */
    } else {
        helper_debug.ina_ok = 0U;
    }
#else
    (void)now_ms;
#endif
}

static void report_tick(uint32_t now_ms) {
    if ((uint32_t)(now_ms - last_report_ms) < PI_REPORT_PERIOD_MS) return;
    last_report_ms = now_ms;
    if (huart1.gState != HAL_UART_STATE_READY) return;     /* the last report is still going out */
    char payload[48];
    snprintf(payload, sizeof(payload), "%u,%u,%ld,%u,%u,%u", (unsigned int)++report_sequence, (unsigned int)helper_debug.bus_mv,
             (long)helper_debug.current_ma, (unsigned int)helper_debug.estop,
             helper_debug.lora_state >= LORA_IDLE ? 1U : 0U, (unsigned int)helper_debug.alarm);
    size_t length = PiLink_Encode(pi_tx, sizeof(pi_tx), "AUX", payload);
    if (length > 0U) (void)HAL_UART_Transmit_IT(&huart1, (uint8_t *)pi_tx, (uint16_t)length);
}

void App_Init(void) {
    memset((void *)&helper_debug, 0, sizeof(helper_debug));
    PiLink_Init(&pi);
    IbusSensor_Init(&sensor);
    relays(false);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, CAM_CENTER_US);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    lora_init();
    last_ina_ms = HAL_GetTick() - INA226_RETRY_MS;          /* look for the INA226 on the first pass */

    HAL_UART_Receive_IT(&huart1, &pi_rx_byte, 1U);
    HAL_HalfDuplex_EnableReceiver(&huart3);
    HAL_UART_Receive_IT(&huart3, &ibus_rx_byte, 1U);
}

void App_Tick(void) {
    ibus_answer();
    uint32_t now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - last_fast_ms) >= FAST_PERIOD_MS) {
        last_fast_ms = now_ms;
        fast_tick(now_ms);
    }
    ina_tick(now_ms);
    ibus_answer();          /* the I2C read above takes about a millisecond */
    report_tick(now_ms);
    lora_tick(now_ms);
}

void App_UartRxComplete(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
        PiLink_FeedByte(&pi, pi_rx_byte, HAL_GetTick());
        HAL_UART_Receive_IT(&huart1, &pi_rx_byte, 1U);
    } else if (huart == &huart3) {
        /* A poll that arrives before the last answer has left is dropped: the receiver asks again. */
        uint8_t reply[IBUS_SENSOR_REPLY_MAX];
        uint8_t length = IbusSensor_FeedByte(&sensor, ibus_rx_byte, reply);
        if (length > 0U && ibus_reply_length == 0U) {
            memcpy(ibus_reply, reply, length);
            ibus_reply_length = length;
        }
        HAL_UART_Receive_IT(&huart3, &ibus_rx_byte, 1U);
    }
}

void App_UartError(UART_HandleTypeDef *huart) {
    /* An overrun makes HAL abort the interrupt reception: start it again, or the port stays deaf
       until the next reset. */
    if (huart == &huart1) (void)HAL_UART_Receive_IT(&huart1, &pi_rx_byte, 1U);
    else if (huart == &huart3) (void)HAL_UART_Receive_IT(&huart3, &ibus_rx_byte, 1U);
}
