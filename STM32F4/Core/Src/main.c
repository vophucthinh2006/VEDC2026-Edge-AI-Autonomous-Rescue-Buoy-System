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
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "actuators.h"
#include "battery.h"
#include "buzzer.h"
#include "lights.h"
#include "app_config.h"
#include "control.h"
#include "gps_nmea.h"
#include "imu.h"
#include "imu_watchdog.h"
#include "i2c_bus.h"
#include "hw_test.h"
#include "lora_beacon.h"
#include "uart_protocol.h"
#include <stdio.h>
#include <string.h>
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
static uint8_t pi_rx_byte, gps_rx_byte;
static pi_command_t pi_command;
static gps_state_t gps;
static bno055_euler_t imu;
static controller_t control;
imu_watchdog_t imu_watchdog;   /* not static: read live over ST-LINK */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if !PI_LINK_TEST_ENABLED
static void send_telemetry(uint16_t sequence)
{
  char payload[128];
  uint32_t now = HAL_GetTick();
  /* Same freshness rule as Control_Tick: a good read within IMU_TIMEOUT_MS. Not imu.valid, which a
     single failed I2C read clears for one tick: the Pi stopped the boat on it (safety.py imu_stale)
     and the front motor braked, while the F407 itself stayed armed. */
  bool imu_ok = control.last_imu_ms != 0U && (uint32_t)(now - control.last_imu_ms) <= IMU_TIMEOUT_MS;

  snprintf(payload, sizeof(payload), "%u,%.2f,%.2f,%.2f,%u,%u", sequence, imu.pitch_deg, imu.roll_deg, imu.heading_deg, imu_ok ? 1U : 0U, control.overturned ? 1U : 0U);
  Protocol_Send(&huart4, "IMU", payload);
  snprintf(payload, sizeof(payload), "%u,%.6f,%.6f,%u,%.1f,%.2f,%.1f", sequence, gps.latitude_deg, gps.longitude_deg, gps.fix, gps.hdop, gps.speed_mps, gps.course_deg);
  Protocol_Send(&huart4, "GPS", payload);
  /* armed and auto let the Pi sweep its camera while the boat waits; with no RC transmitter auto is
     always 1. Then the presses of the victim's confirm button (PE4) since boot, and the battery's
     charge in % (-1 unknown) after its voltage in the first field. */
  snprintf(payload, sizeof(payload), "%u,%.2f,%u,%u,%u,%u,1,%u,%d", sequence, (double)Battery_Volts(), control.motor_fault ? 1U : 0U,
           control.estop ? 1U : 0U, pi_command.pi_link_ok ? 1U : 0U, control.armed ? 1U : 0U, (unsigned)control.confirm.presses,
           Battery_Percent());
  Protocol_Send(&huart4, "SYS", payload);
}
#endif

#if !HW_TEST_ENABLED
/* Events for the shore station over LoRa: a person the Pi reported, the boat overturned (again
   every TILT_REPEAT_MS while it stays so), the confirm button pressed. */
static void report_events(uint32_t now)
{
  static bool was_overturned;
  static uint32_t last_tilt_ms;
  static uint16_t presses_seen;
  if (control.overturned && (!was_overturned || (uint32_t)(now - last_tilt_ms) >= TILT_REPEAT_MS))
  {
    LoraBeacon_ReportTilt(imu.roll_deg, imu.pitch_deg, now);
    last_tilt_ms = now;
  }
  was_overturned = control.overturned;
  if (control.confirm.presses != presses_seen)
  {
    presses_seen = control.confirm.presses;
    LoraBeacon_ReportConfirm(presses_seen, now);
  }
}
#endif
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  Protocol_Init(&pi_command);
  GPS_Init(&gps);
  Control_Init(&control);
  Lights_Init();           /* off until the shore station sends the admin's command */
  LoraBeacon_Init(&hspi2, &gps, &imu, &control); /* SPI2 + NSS PB12 + RST PC4, non-blocking */
#if HW_TEST_ENABLED
  HwTest_Init();
#else
  Actuators_Init();
  Actuators_StartEscs();   /* stop signal from boot so the ESCs arm */
  Buzzer_Init();
  Buzzer_Beep(1U);         /* firmware is up */
#endif
  ImuWatchdog_Init(&imu_watchdog, IMU_Init(), HAL_GetTick());   /* a failed init is tried again */
  Battery_Init(&hi2c1);    /* INA226 on the BNO055's bus */

  HAL_UART_Receive_IT(&huart4, &pi_rx_byte, 1U);
  HAL_UART_Receive_IT(&huart3, &gps_rx_byte, 1U);
  /* USART2 (PA2, half duplex) was the FS-iA6B's iBUS: there is no RC transmitter any more, nothing
     reads it. CubeMX still sets it up; remove it in STM32F4.ioc to free the pin. */

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
    LoraBeacon_Tick(now);

    if ((uint32_t)(now - last_control) >= CONTROL_PERIOD_MS)
    {
      last_control = now;
      /* A stuck I2C bus is recovered instead of read: each transfer on it waited 25 ms for nothing. */
      bool bus_ok = I2cBus_Ready(&hi2c1, now);
      bool imu_read = bus_ok && IMU_Read(&imu);
      if (imu_read) control.last_imu_ms = now;
      else if (!bus_ok) imu.valid = false;
      ImuWatchdog_OnRead(&imu_watchdog, imu_read, now);
      if (bus_ok) Battery_Tick(now);   /* same bus as the IMU, after it, never during */
#if !HW_TEST_ENABLED
      /* The BNO055 lost, or reset by itself: started again. That blocks ~0.7..2 s, so every motor is
         stopped first (HwTest never does it: its motor time limit would stall meanwhile). A lost
         IMU has disarmed the boat already; only BENCH_IGNORE_IMU drives on without one, and the
         stand is then left driving. A sensor that reset itself reads zeros that look fresh: that
         one is stopped and started again whatever the boat does. */
      if (bus_ok && ImuWatchdog_CheckDue(&imu_watchdog, now))
        ImuWatchdog_OnModeCheck(&imu_watchdog, IMU_SensorReset(), now);
      bool driving = control.operating_mode == 'A';
      if (ImuWatchdog_ReinitDue(&imu_watchdog, now) && (!driving || imu_watchdog.cause == IMU_CAUSE_MODE))
      {
        Control_Stop(&control);
        if (!bus_ok) I2cBus_Recover(&hi2c1);
        bool ok = IMU_Init();
        now = HAL_GetTick();
        ImuWatchdog_OnReinit(&imu_watchdog, ok, now);
        imu.valid = false;     /* fresh again only after the next good read */
      }
#endif
#if HW_TEST_ENABLED
      HwTest_Tick(now);
#else
      control.hold = LoraBeacon_Hold();
      Control_Tick(&control, &pi_command, &imu, now);
      Camera_Tick(&pi_command, now);
      Buzzer_Tick(now);
      report_events(now);
#endif
      /* SOS output (old PD13) has no pin in Pinout (Hy) yet. */

      /* Every TXD queued since the last tick: several come back to back when people are confirmed together. */
      txd_event_t txd;
      while (TxdQueue_Pop(&pi_command.txd, &txd))
      {
        char ack[40];
        snprintf(ack, sizeof(ack), "%u,TXD,ACCEPTED", txd.sequence);
        Protocol_Send(&huart4, "ACK", ack);
        LoraBeacon_SetVictims(txd.victims, now);
        if (txd.victim_report)
          LoraBeacon_ReportVictim(txd.has_pos, txd.lat_e6, txd.lon_e6, txd.risk, txd.confidence_pct, now);
      }
    }

    if ((uint32_t)(now - last_telemetry) >= TELEMETRY_PERIOD_MS)
    {
      last_telemetry = now;
#if PI_LINK_TEST_ENABLED
      char payload[48];
      snprintf(payload, sizeof(payload), "%u,%lu,HELLO_PI", ++sequence, (unsigned long)now);
      Protocol_Send(&huart4, "TST", payload);
#else
      send_telemetry(++sequence);
      /* Diagnostics for the dashboard, sent over LoRa every LORA_DIAG_PERIOD_MS (App/diag_frame.h). */
      diag_t diag = { 0 };
      char reason[sizeof(pi_command.pi_reason)];
      memcpy(reason, pi_command.pi_reason, sizeof(reason));   /* the UART interrupt writes it */
      reason[sizeof(reason) - 1U] = '\0';
      bool pi_heard = pi_command.pi_link_ok && (uint32_t)(now - pi_command.last_hbt_ms) <= DIAG_PI_SILENT_MS;
      Diag_SetPi(&diag, pi_heard, reason, pi_command.pi_mission, pi_command.pi_wp, pi_command.pi_wp_total);
      diag.has_fps = pi_heard && pi_command.pi_fps_x10 >= 0;
      diag.fps_x10 = diag.has_fps ? (uint16_t)pi_command.pi_fps_x10 : 0U;
      diag.arm_block = heading_debug.arm_block;
      diag.drive_block = heading_debug.block;
      diag.has_battery = battery_debug.ok != 0U;
      diag.volts = battery_debug.volts;
      diag.amps = battery_debug.amps;
      diag.i2c_recoveries = i2c_bus_debug.recoveries;
      diag.imu_reinits = imu_watchdog.attempts;
      LoraBeacon_SetDiag(&diag);
      /* Route from the dashboard over LoRa: one $RTE line a period until the Pi's $HBT confirms it. */
      char route_line[180];
      LoraBeacon_SetPiRouteRev(pi_command.route_rev);
      if (LoraBeacon_NextPiRouteLine(sequence, now, route_line, sizeof(route_line)))
        Protocol_Send(&huart4, "RTE", route_line);
      /* Once a second the shore link and the way home ($LNK): the Pi takes the boat home when it is lost. */
      if (sequence % (1000U / TELEMETRY_PERIOD_MS) == 0U &&
          LoraBeacon_LinkLine(sequence, now, route_line, sizeof(route_line)) > 0U)
        Protocol_Send(&huart4, "LNK", route_line);
#endif
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

  if (huart == &huart4)
  {
    Protocol_FeedByte(&pi_command, pi_rx_byte, now);
    HAL_UART_Receive_IT(&huart4, &pi_rx_byte, 1U);
  }
  else if (huart == &huart3)
  {
    GPS_FeedByte(&gps, gps_rx_byte, now);
    HAL_UART_Receive_IT(&huart3, &gps_rx_byte, 1U);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* Clear the error flags first (SR read, then DR read). HAL only reads DR when RXNE is set: an
     overrun caught with RXNE already clear left ORE set, and re-arming RXNEIE on it fired the
     interrupt again at once, for ever. Seen on the stand (2026-10-18): stuck in USART3, the GPS,
     with SR 0xD8; the main loop never ran again, no telemetry, no PE4. */
  __HAL_UART_CLEAR_OREFLAG(huart);
  /* An overrun makes HAL abort the interrupt reception. Re-arm the GPS byte so the
     fix source does not stay silent until the next reset (BUSY if HAL kept it armed). */
  if (huart == &huart3) (void)HAL_UART_Receive_IT(&huart3, &gps_rx_byte, 1U);
  /* Same for the Pi link: without this one overrun leaves UART4 deaf until reset. */
  else if (huart == &huart4) (void)HAL_UART_Receive_IT(&huart4, &pi_rx_byte, 1U);
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
