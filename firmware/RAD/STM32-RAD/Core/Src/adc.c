/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */

uint8_t adcInitialized;
uint8_t adc_buf[ADC_BUF_LEN];
uint8_t adc_state[ADC_BUF_LEN];
uint8_t gpio_in_buf[ADC_BUF_LEN];

// Missing a callback initialization

// REVISIT EVERYTHING WITH THIS NEWFOUND KNOWLEDGE!!!!!

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}
/* ADC2 init function */
void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA2     ------> ADC1_IN2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspInit 0 */

  /* USER CODE END ADC2_MspInit 0 */
    /* ADC2 clock enable */
    __HAL_RCC_ADC2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC2 GPIO Configuration
    PA3     ------> ADC2_IN3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA2     ------> ADC1_IN2
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC2_CLK_DISABLE();

    /**ADC2 GPIO Configuration
    PA3     ------> ADC2_IN3
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);

  /* USER CODE BEGIN ADC2_MspDeInit 1 */

  /* USER CODE END ADC2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

ADC_STATUS ADC_Initialize() {

	MX_GPIO_Init();
	HAL_ADC_Init(*hadc1); // the problem here is I do not know what goes on the inside of this function, but I know there should be something
	// in the brackets here
	HAL_ADC_Init(*hadc2);

	// This is probably where you run all the starter code too for the analog watchdog


	adcInitialized = 1;

	return ADC_OK;
}

ADC_STATUS ADC_Deinitialize() {
	if (!adcInitialized) {
		return ADC_ERROR_NOT_INITIALIZED;
	}

	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);
//parameters are a problem again
	HAL_ADC_DeInit(*hadc1);
	HAL_ADC_DeInit(*hadc2);


	return ADC_OK;
}

ADC_STATUS ADC_GetValue(ADC_NUMBER num, uint16_t *pointer) {
	if (!adcInitialized) {
		return ADC_ERROR_NOT_INITIALIZED;

	}

	if (num == 0) {
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1);
		adc_buf[0] = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

	} else if (num == 1) {
		HAL_ADC_Start(&hadc2);
		HAL_ADC_PollForConversion(&hadc2, 1);
		adc_buf[1] = HAL_ADC_GetValue(&hadc2);
		HAL_ADC_Stop(&hadc2);
	} else {
		return ADC_ERROR_INVALID_ARGUMENT;
	}

	return ADC_OK;

}

ADC_STATUS ADC_GetState(ADC_NUMBER num, ADC_STATE *state) {
	if (!adcInitialized) {
		return ADC_ERROR_NOT_INITIALZED;

	}

	if (num == 0) {
		HAL_ADC_Start(&hadc1);
		adc_state[0] = HAL_ADC_GetState(*hadc1);
		HAL_ADC_Stop(&hadc1);

	} else if (num == 1) {
		HAL_ADC_Start(&hadc2);
		adc_state[0] = HAL_ADC_GetState(*hadc2);
		HAL_ADC_Stop(&hadc2);

	} else {
		return ADC_ERROR_INVALID_ARGUMENT;
	}

	return ADC_OK;
}

ADC_STATUS ADC_ConfigureWatchdogThreshold(ADC_NUMBER num, ADC_THRESHOLD threshold, uint16_t value) {
	if (num == 0) {
		HAL_ADC_Start(&hadc1);
		HAL_ADC_AnalogWDGConfig(*hadc1, value); // (*ANalogWDGConfig)
		HAL_ADC_Stop(&hadc1);

	} else if (num == 1) {
		HAL_ADC_AnalogWDGConfig(*hadc2, value); // same missing parameter that I need to figure out what it is
	}

	AnalogWDCConfig.WatchdogMode = ADC_AMALOGWATCHDOG_ALL_REG;
	AnalogWDGConfig.ITMode = ENABLE;



	// the idea is right here, but we are missing a parameter i believe
	// the other problem is there's nothing to set this to the lower or the upper threshold


	return ADC_OK;
}

ADC_STATUS ADC_RegisterWatchdogCallback(void (*fcn)(ADC_NUMBER *num)) {

	if (!adcInitialized) {
		return ADC_ERROR_NOT_INITIALIZED;
	}

	if (!fcn) {
		return ADC_ERROR_INVALID_ARGUMENT;
	}

	callback_function = fcn;

	return ADC_OK;

}


ADC_STATUS ADC_ConfigureInterrupt(ADC_INTERRUPT interrupt) {
	// Do we need an ADC Number on this?  I assume we might

	HAL_ADC_AnalogWDGConfig(interrupt);


} return ADC_OK;

/* USER CODE END 1 */
