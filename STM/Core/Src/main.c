/* CubeMX must generate clock/peripheral initialization functions and handles declared by main.h. */
#include "main.h"
#include "actuators.h"
#include "app_config.h"
#include "bno055.h"
#include "control.h"
#include "gps_nmea.h"
#include "ibus.h"
#include "uart_protocol.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

static uint8_t pi_rx_byte, gps_rx_byte, ibus_rx_byte;
static pi_command_t pi_command;
static ibus_state_t ibus;
static gps_state_t gps;
static bno055_euler_t imu;
static controller_t control;
static actuator_driver_t actuators;

static void send_telemetry(uint16_t sequence) {
    char payload[128];
    uint32_t now = HAL_GetTick();
    bool imu_ok = imu.valid && (uint32_t)(now - control.last_imu_ms) <= IMU_TIMEOUT_MS;
    snprintf(payload, sizeof(payload), "%u,%.2f,%.2f,%.2f,%u,%u", sequence, imu.pitch_deg, imu.roll_deg, imu.heading_deg, imu_ok ? 1U : 0U, control.overturned ? 1U : 0U);
    Protocol_Send(&huart1, "IMU", payload);
    snprintf(payload, sizeof(payload), "%u,%.6f,%.6f,%u,%.1f,%.2f,%.1f", sequence, gps.latitude_deg, gps.longitude_deg, gps.fix, gps.hdop, gps.speed_mps, gps.course_deg);
    Protocol_Send(&huart1, "GPS", payload);
    snprintf(payload, sizeof(payload), "%u,0.0,%u,%u,%u", sequence, control.motor_fault ? 1U : 0U, control.estop ? 1U : 0U, pi_command.pi_link_ok ? 1U : 0U);
    Protocol_Send(&huart1, "SYS", payload);
}

int main(void) {
    HAL_Init(); SystemClock_Config();
    MX_GPIO_Init(); MX_I2C1_Init(); MX_TIM3_Init(); MX_TIM4_Init(); MX_USART1_UART_Init(); MX_USART3_UART_Init(); MX_USART6_UART_Init();
    Protocol_Init(&pi_command); IBUS_Init(&ibus); GPS_Init(&gps); Control_Init(&control); Actuators_Init(&actuators, &htim3, &htim4);
    (void)BNO055_Init(&hi2c1);
    HAL_UART_Receive_IT(&huart1, &pi_rx_byte, 1U); HAL_UART_Receive_IT(&huart3, &gps_rx_byte, 1U); HAL_UART_Receive_IT(&huart6, &ibus_rx_byte, 1U);
    uint32_t last_control = 0U, last_telemetry = 0U; uint16_t sequence = 0U;
    while (1) {
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - last_control) >= CONTROL_PERIOD_MS) {
            last_control = now;
            if (BNO055_ReadEuler(&hi2c1, &imu)) control.last_imu_ms = now;
            Control_Tick(&control, &pi_command, &ibus, &imu, &actuators, now);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, pi_command.sos_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
            if (pi_command.txd_pending) {
                char ack[40];
                snprintf(ack, sizeof(ack), "%u,TXD,ACCEPTED", pi_command.txd_sequence);
                Protocol_Send(&huart1, "ACK", ack);
                pi_command.txd_pending = false;
            }
        }
        if ((uint32_t)(now - last_telemetry) >= TELEMETRY_PERIOD_MS) { last_telemetry = now; send_telemetry(++sequence); }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    uint32_t now = HAL_GetTick();
    if (huart == &huart1) { Protocol_FeedByte(&pi_command, pi_rx_byte, now); HAL_UART_Receive_IT(&huart1, &pi_rx_byte, 1U); }
    else if (huart == &huart3) { GPS_FeedByte(&gps, gps_rx_byte, now); HAL_UART_Receive_IT(&huart3, &gps_rx_byte, 1U); }
    else if (huart == &huart6) { IBUS_FeedByte(&ibus, ibus_rx_byte, now); HAL_UART_Receive_IT(&huart6, &ibus_rx_byte, 1U); }
}
