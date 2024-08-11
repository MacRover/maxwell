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
#define LED_PIN_Pin GPIO_PIN_13
#define LED_PIN_GPIO_Port GPIOC
#define WRIST_STEP_Pin GPIO_PIN_12
#define WRIST_STEP_GPIO_Port GPIOB
#define WRIST_DIR_Pin GPIO_PIN_13
#define WRIST_DIR_GPIO_Port GPIOB
#define GRIPPER_STEP_Pin GPIO_PIN_9
#define GRIPPER_STEP_GPIO_Port GPIOA
#define GRIPPER_DIR_Pin GPIO_PIN_10
#define GRIPPER_DIR_GPIO_Port GPIOA
#define BASE_STEP_Pin GPIO_PIN_3
#define BASE_STEP_GPIO_Port GPIOB
#define BASE_DIR_Pin GPIO_PIN_4
#define BASE_DIR_GPIO_Port GPIOB
#define ELBOW_STEP_Pin GPIO_PIN_8
#define ELBOW_STEP_GPIO_Port GPIOB
#define ELBOW_DIR_Pin GPIO_PIN_9
#define ELBOW_DIR_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
