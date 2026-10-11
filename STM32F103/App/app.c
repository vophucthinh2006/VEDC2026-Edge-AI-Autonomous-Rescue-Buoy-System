#include "app.h"
#include "app_config.h"
#include "helper_logic.h"
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
#include <string.h>

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

static helper_logic_t logic;          /* every decision: App/helper_logic.c */
static helper_inputs_t inputs;
static uint32_t last_fast_ms, last_ina_ms, last_report_ms;
static uint32_t lora_next_ms, lora_start_ms, lora_poll_ms;

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
        HAL_NVIC_DisableIRQ(USART1_IRQn);
        bool status_seen = pi.status_seen;
        uint32_t last_status_ms = pi.last_status_ms;
        memcpy(status, pi.status, sizeof(status));
        HAL_NVIC_EnableIRQ(USART1_IRQn);

        char frame[LORA_FRAME_SIZE];
        size_t length = HelperLogic_Beacon(&logic, status, status_seen, last_status_ms, now_ms, helper_debug.lora_seq, frame, sizeof(frame));
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
    inputs.estop_pressed = HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == ESTOP_PRESSED_LEVEL;
    inputs.lora_ok = helper_debug.lora_state >= LORA_IDLE;
    HelperLogic_Fast(&logic, &pi, &inputs, now_ms);
    if (HelperLogic_TakeBeaconRequest(&logic)) lora_next_ms = now_ms;

    const helper_outputs_t *out = &logic.out;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, out->servo_us);
    relays(out->relays_on);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, out->led_on ? GPIO_PIN_RESET : GPIO_PIN_SET);   /* lit when low */

    helper_debug.pi_ok = out->pi_ok ? 1U : 0U;
    helper_debug.pi_rx_ok = pi.rx_ok;
    helper_debug.pi_rx_bad = pi.rx_bad;
    helper_debug.servo_us = out->servo_us;
    helper_debug.victims = out->victims;
    helper_debug.alarm = out->relays_on ? 1U : 0U;
    helper_debug.estop = out->estop ? 1U : 0U;
}

static void ina_tick(uint32_t now_ms) {
#if INA226_ENABLED
    uint32_t period = helper_debug.ina_ok ? INA226_PERIOD_MS : INA226_RETRY_MS;
    if ((uint32_t)(now_ms - last_ina_ms) < period) return;
    last_ina_ms = now_ms;
    if (!helper_debug.ina_ok) {
        helper_debug.ina_ok = INA226_Init(&ina) ? 1U : 0U;
        inputs.ina_ok = helper_debug.ina_ok != 0U;
        return;
    }
    uint16_t bus_mv;
    int32_t current_ma;
    if (INA226_Read(&ina, &bus_mv, &current_ma)) {
        helper_debug.bus_mv = inputs.bus_mv = bus_mv;
        helper_debug.current_ma = inputs.current_ma = current_ma;
        sensor.value = (uint16_t)(bus_mv / 10U);      /* the handset shows 0.01 V per count */
    } else {
        helper_debug.ina_ok = 0U;
        inputs.ina_ok = false;
    }
#else
    (void)now_ms;
#endif
}

static void report_tick(uint32_t now_ms) {
    if ((uint32_t)(now_ms - last_report_ms) < PI_REPORT_PERIOD_MS) return;
    last_report_ms = now_ms;
    if (huart1.gState != HAL_UART_STATE_READY) return;     /* the last report is still going out */
    size_t length = HelperLogic_Report(&logic, &inputs, pi_tx, sizeof(pi_tx));
    if (length > 0U) (void)HAL_UART_Transmit_IT(&huart1, (uint8_t *)pi_tx, (uint16_t)length);
}

void App_Init(void) {
    memset((void *)&helper_debug, 0, sizeof(helper_debug));
    PiLink_Init(&pi);
    HelperLogic_Init(&logic);
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
    if ((uint32_t)(now_ms - last_fast_ms) >= HELPER_FAST_PERIOD_MS) {
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
