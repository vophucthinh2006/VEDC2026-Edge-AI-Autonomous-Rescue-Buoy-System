/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BNO055_INT_Pin GPIO_PIN_2
#define BNO055_INT_GPIO_Port GPIOE
#define BNO055_INT_EXTI_IRQn EXTI2_IRQn
#define ESTOP_Pin GPIO_PIN_4
#define ESTOP_GPIO_Port GPIOE
#define ESTOP_EXTI_IRQn EXTI4_IRQn
#define RELAY1_Pin GPIO_PIN_0
#define RELAY1_GPIO_Port GPIOC
#define RELAY2_Pin GPIO_PIN_2
#define RELAY2_GPIO_Port GPIOC
#define LORA_RST_Pin GPIO_PIN_4
#define LORA_RST_GPIO_Port GPIOC
#define LORA_DIO0_Pin GPIO_PIN_11
#define LORA_DIO0_GPIO_Port GPIOE
#define LORA_DIO0_EXTI_IRQn EXTI15_10_IRQn
#define LORA_DIO1_Pin GPIO_PIN_12
#define LORA_DIO1_GPIO_Port GPIOE
#define LORA_DIO1_EXTI_IRQn EXTI15_10_IRQn
#define LORA_DIO2_Pin GPIO_PIN_13
#define LORA_DIO2_GPIO_Port GPIOE
#define LORA_DIO2_EXTI_IRQn EXTI15_10_IRQn
#define LORA_NSS_Pin GPIO_PIN_12
#define LORA_NSS_GPIO_Port GPIOB
#define BNO055_RST_Pin GPIO_PIN_6
#define BNO055_RST_GPIO_Port GPIOC
#define GPS_SW_LED_Pin GPIO_PIN_9
#define GPS_SW_LED_GPIO_Port GPIOC
#define GPS_SW_Pin GPIO_PIN_8
#define GPS_SW_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
