/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

// LS_STATUS ENUM -------------------------

typedef enum {
	LS_OK = 0,
	LS_ERROR_NOT_INITIALIZED,
	LS_ERROR_HAL,
	LS_ERROR_PIN,
	LS_ERROR_INVALID_ARGUMENT

} LS_STATUS;

// LS_NUMBER ENUM -------------------------

typedef enum {
	LS_1 = 0,
	LS_2

} LS_NUMBER;

typedef enum {
	LS_STATE_PRESSED = 0,
	LS_STATE_RELEASED

} LS_STATE;

LS_STATUS LS_Initialize(void);
LS_STATUS LS_GetState(LS_NUMBER num, LS_STATE *state);
LS_STATUS LS_RegisterPressedCallback(void (*fcn)(LS_NUMBER *num));
LS_STATUS LS_RegisterReleasedCallback(void (*fcn)(LS_NUMBER *num));
LS_STATUS LS_Deinitialize(void);


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

