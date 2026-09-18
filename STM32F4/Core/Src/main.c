/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "actuators.h"
#include "app_config.h"
#include "control.h"
#include "gps_nmea.h"
#include "ibus.h"
#include "imu.h"
#include "uart_protocol.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BNO055_READ_PERIOD_MS  10U   /* fusion output data rate 100 Hz (Table 3-14) */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t pi_rx_byte, gps_rx_byte, ibus_rx_byte;
static pi_command_t pi_command;
static ibus_state_t ibus;
static gps_state_t gps;
static bno055_euler_t imu;
static controller_t control;
static actuator_driver_t actuators;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void send_telemetry(uint16_t sequence)
{
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
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  Protocol_Init(&pi_command);
  IBUS_Init(&ibus);
  GPS_Init(&gps);
  Control_Init(&control);
  Actuators_Init(&actuators, &htim3, &htim4);
  (void)IMU_Init();

  HAL_UART_Receive_IT(&huart1, &pi_rx_byte, 1U);
  HAL_UART_Receive_IT(&huart3, &gps_rx_byte, 1U);
  HAL_UART_Receive_IT(&huart6, &ibus_rx_byte, 1U);

  uint32_t last_control = 0U, last_telemetry = 0U;
  uint16_t sequence = 0U;
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - last_control) >= CONTROL_PERIOD_MS)
    {
      last_control = now;
      if (IMU_Read(&imu)) control.last_imu_ms = now;
      Control_Tick(&control, &pi_command, &ibus, &imu, &actuators, now);
      HAL_GPIO_WritePin(SOS_GPIO_Port, SOS_Pin, pi_command.sos_on ? GPIO_PIN_SET : GPIO_PIN_RESET);

      if (pi_command.txd_pending)
      {
        char ack[40];
        snprintf(ack, sizeof(ack), "%u,TXD,ACCEPTED", pi_command.txd_sequence);
        Protocol_Send(&huart1, "ACK", ack);
        pi_command.txd_pending = false;
      }
    }

    if ((uint32_t)(now - last_telemetry) >= TELEMETRY_PERIOD_MS)
    {
      last_telemetry = now;
      send_telemetry(++sequence);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint32_t now = HAL_GetTick();

  if (huart == &huart1)
  {
    Protocol_FeedByte(&pi_command, pi_rx_byte, now);
    HAL_UART_Receive_IT(&huart1, &pi_rx_byte, 1U);
  }
  else if (huart == &huart3)
  {
    GPS_FeedByte(&gps, gps_rx_byte, now);
    HAL_UART_Receive_IT(&huart3, &gps_rx_byte, 1U);
  }
  else if (huart == &huart6)
  {
    IBUS_FeedByte(&ibus, ibus_rx_byte, now);
    HAL_UART_Receive_IT(&huart6, &ibus_rx_byte, 1U);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
