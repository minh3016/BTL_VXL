/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define SW1_Pin GPIO_PIN_3
#define SW1_GPIO_Port GPIOA
#define SW2_Pin GPIO_PIN_4
#define SW2_GPIO_Port GPIOA
#define SW3_Pin GPIO_PIN_5
#define SW3_GPIO_Port GPIOA
#define BUZZ_Pin GPIO_PIN_6
#define BUZZ_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_7
#define LED_GPIO_Port GPIOA
#define D7_Pin GPIO_PIN_14
#define D7_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_15
#define D6_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_8
#define D5_GPIO_Port GPIOA
#define D4_Pin GPIO_PIN_9
#define D4_GPIO_Port GPIOA
#define EN_Pin GPIO_PIN_10
#define EN_GPIO_Port GPIOA
#define RW_Pin GPIO_PIN_11
#define RW_GPIO_Port GPIOA
#define RS_Pin GPIO_PIN_12
#define RS_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define SW1 SW1_GPIO_Port,SW1_Pin
#define SW2 SW2_GPIO_Port,SW2_Pin
#define SW3 SW3_GPIO_Port,SW3_Pin
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
