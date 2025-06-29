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
#define IN_FAULT_CARD_0_Pin GPIO_PIN_0
#define IN_FAULT_CARD_0_GPIO_Port GPIOA
#define IN_FAULT_CARD_1_Pin GPIO_PIN_1
#define IN_FAULT_CARD_1_GPIO_Port GPIOA
#define IN_FAULT_CARD_2_Pin GPIO_PIN_2
#define IN_FAULT_CARD_2_GPIO_Port GPIOA
#define IN_FAULT_CARD_3_Pin GPIO_PIN_3
#define IN_FAULT_CARD_3_GPIO_Port GPIOA
#define PGOOD_CARD_0_Pin GPIO_PIN_4
#define PGOOD_CARD_0_GPIO_Port GPIOA
#define PGOOD_CARD_1_Pin GPIO_PIN_5
#define PGOOD_CARD_1_GPIO_Port GPIOA
#define PGOOD_CARD_2_Pin GPIO_PIN_6
#define PGOOD_CARD_2_GPIO_Port GPIOA
#define PGOOD_CARD_3_Pin GPIO_PIN_7
#define PGOOD_CARD_3_GPIO_Port GPIOA
#define OUT_FAULT_5_Pin GPIO_PIN_0
#define OUT_FAULT_5_GPIO_Port GPIOB
#define OUT_FAULT_4_Pin GPIO_PIN_1
#define OUT_FAULT_4_GPIO_Port GPIOB
#define OUT_FAULT_3_Pin GPIO_PIN_2
#define OUT_FAULT_3_GPIO_Port GPIOB
#define OUT_FAULT_2_Pin GPIO_PIN_12
#define OUT_FAULT_2_GPIO_Port GPIOB
#define OUT_FAULT_1_Pin GPIO_PIN_13
#define OUT_FAULT_1_GPIO_Port GPIOB
#define OUT_FAULT_0_Pin GPIO_PIN_14
#define OUT_FAULT_0_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_15
#define LED_GPIO_Port GPIOB
#define SM_ALERT_Pin GPIO_PIN_3
#define SM_ALERT_GPIO_Port GPIOB
#define EN_CARD_0_Pin GPIO_PIN_4
#define EN_CARD_0_GPIO_Port GPIOB
#define EN_CARD_1_Pin GPIO_PIN_5
#define EN_CARD_1_GPIO_Port GPIOB
#define EN_CARD_2_Pin GPIO_PIN_8
#define EN_CARD_2_GPIO_Port GPIOB
#define EN_CARD_3_Pin GPIO_PIN_9
#define EN_CARD_3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
