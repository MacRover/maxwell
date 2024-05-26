/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <string.h>

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
#define LIMITSWITCH_1_Pin GPIO_PIN_0
#define LIMITSWITCH_1_GPIO_Port GPIOA
#define LIMITSWITCH_1_EXTI_IRQn EXTI0_IRQn
#define LIMITSWITCH_2_Pin GPIO_PIN_1
#define LIMITSWITCH_2_GPIO_Port GPIOA
#define LIMITSWITCH_2_EXTI_IRQn EXTI1_IRQn
#define DRIVER_CS_Pin GPIO_PIN_4
#define DRIVER_CS_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_2
#define LED_RED_GPIO_Port GPIOB
#define ENCODER_CS_Pin GPIO_PIN_12
#define ENCODER_CS_GPIO_Port GPIOB
#define DRIVER_SG_TEST_Pin GPIO_PIN_9
#define DRIVER_SG_TEST_GPIO_Port GPIOA
#define DRIVER_ENN_Pin GPIO_PIN_10
#define DRIVER_ENN_GPIO_Port GPIOA
#define DRIVER_ST_ALONE_Pin GPIO_PIN_15
#define DRIVER_ST_ALONE_GPIO_Port GPIOA
#define DRIVER_STEP_Pin GPIO_PIN_3
#define DRIVER_STEP_GPIO_Port GPIOB
#define DRIVER_DIR_Pin GPIO_PIN_4
#define DRIVER_DIR_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
