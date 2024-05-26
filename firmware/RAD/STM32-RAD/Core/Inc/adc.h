/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

extern ADC_HandleTypeDef hadc2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);

/* USER CODE BEGIN Prototypes */

typedef enum {
	ADC_OK = 0,
	ADC_ERROR_NOT_INITIALIZED,
	ADC_ERROR_HAL,
	ADC_ERROR_PIN,
	ADC_ERROR_INVALID_ARGUMENT
} ADC_STATUS;

typedef enum {
	ADC_1 = 0,
	ADC_2
} ADC_NUMBER ;

typedef enum {
	ADC_THRESHOLD_HIGH = 0,
	ADC_THRESHOLD_LOW
} ADC_THRESHOLD;

typedef enum {
	ADC_STATE_ABOVE_WINDOW = 0,
	ADC_STATE_WITHIN_WINDOW,
	ADC_STATE_BELOW_WINDOW
} ADC_STATE;

typedef enum {
	ADC_INTERRUPT_ENABLED = 0,
	ADC_INTERRUPT_DISABLED
} ADC_INTERRUPT;

ADC_STATUS ADC_Initialize(void);
ADC_STATUS ADC_GetValue(ADC_NUMBER num, uint16_t *pointer);
ADC_STATUS ADC_GetState(ADC_NUMBER num, ADC_STATE *state);
ADC_STATUS ADC_RegisterWatchdogCallback(void (*fcn)(ADC_NUMBER *num));
ADC_STATUS ADC_ConfigureInterrupt(ADC_INTERRUPT interrupt);
ADC_STATUS ADC_ConfigureWatchdogThreshold(ADC_NUMBER num, ADC_THRESHOLD threshold, uint16_t value);
// get watchdog thresholds???
ADC_STATUS ADC_Deinitialize(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

