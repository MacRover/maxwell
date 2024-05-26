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
// uint8_t adc_buf[ADC_BUF_LEN];
// uint8_t adc_state[ADC_BUF_LEN];
// uint8_t gpio_in_buf[ADC_BUF_LEN];

void (*adc_callback)(ADC_NUMBER *num) = NULL;

ADC_AnalogWDGConfTypeDef AnalogWDGConfig_1 = {0};
ADC_AnalogWDGConfTypeDef AnalogWDGConfig_2 = {0};
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

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
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

  /** Configure Analog WatchDog 1
   */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.HighThreshold = 0;
  AnalogWDGConfig.LowThreshold = 0;
  AnalogWDGConfig.Channel = ADC_CHANNEL_2;
  AnalogWDGConfig.ITMode = DISABLE;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
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
  AnalogWDGConfig_1 = AnalogWDGConfig;
  /* USER CODE END ADC1_Init 2 */
}
/* ADC2 init function */
void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
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

  /** Configure Analog WatchDog 1
   */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.HighThreshold = 0;
  AnalogWDGConfig.LowThreshold = 0;
  AnalogWDGConfig.Channel = ADC_CHANNEL_3;
  AnalogWDGConfig.ITMode = DISABLE;
  if (HAL_ADC_AnalogWDGConfig(&hadc2, &AnalogWDGConfig) != HAL_OK)
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
  AnalogWDGConfig_2 = AnalogWDGConfig;
  /* USER CODE END ADC2_Init 2 */
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (adcHandle->Instance == ADC1)
  {
    /* USER CODE BEGIN ADC1_MspInit 0 */

    /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA2     ------> ADC1_IN2
    */
    GPIO_InitStruct.Pin = ADC_1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(ADC_1_GPIO_Port, &GPIO_InitStruct);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    /* USER CODE BEGIN ADC1_MspInit 1 */

    /* USER CODE END ADC1_MspInit 1 */
  }
  else if (adcHandle->Instance == ADC2)
  {
    /* USER CODE BEGIN ADC2_MspInit 0 */

    /* USER CODE END ADC2_MspInit 0 */
    /* ADC2 clock enable */
    __HAL_RCC_ADC2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC2 GPIO Configuration
    PA3     ------> ADC2_IN3
    */
    GPIO_InitStruct.Pin = ADC_2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(ADC_2_GPIO_Port, &GPIO_InitStruct);

    /* ADC2 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    /* USER CODE BEGIN ADC2_MspInit 1 */

    /* USER CODE END ADC2_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle)
{

  if (adcHandle->Instance == ADC1)
  {
    /* USER CODE BEGIN ADC1_MspDeInit 0 */

    /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA2     ------> ADC1_IN2
    */
    HAL_GPIO_DeInit(ADC_1_GPIO_Port, ADC_1_Pin);

    /* ADC1 interrupt Deinit */
    /* USER CODE BEGIN ADC1:ADC1_2_IRQn disable */
    /**
     * Uncomment the line below to disable the "ADC1_2_IRQn" interrupt
     * Be aware, disabling shared interrupt may affect other IPs
     */
    /* HAL_NVIC_DisableIRQ(ADC1_2_IRQn); */
    /* USER CODE END ADC1:ADC1_2_IRQn disable */

    /* USER CODE BEGIN ADC1_MspDeInit 1 */

    /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if (adcHandle->Instance == ADC2)
  {
    /* USER CODE BEGIN ADC2_MspDeInit 0 */

    /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC2_CLK_DISABLE();

    /**ADC2 GPIO Configuration
    PA3     ------> ADC2_IN3
    */
    HAL_GPIO_DeInit(ADC_2_GPIO_Port, ADC_2_Pin);

    /* ADC2 interrupt Deinit */
    /* USER CODE BEGIN ADC2:ADC1_2_IRQn disable */
    /**
     * Uncomment the line below to disable the "ADC1_2_IRQn" interrupt
     * Be aware, disabling shared interrupt may affect other IPs
     */
    /* HAL_NVIC_DisableIRQ(ADC1_2_IRQn); */
    /* USER CODE END ADC2:ADC1_2_IRQn disable */

    /* USER CODE BEGIN ADC2_MspDeInit 1 */

    /* USER CODE END ADC2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{
  static ADC_NUMBER num;
  if (hadc == &hadc1)
  {
    num = ADC_1;
  }
  else if (hadc == &hadc2)
  {
    num = ADC_2;
  }

  (*adc_callback)(&num);
}

ADC_STATUS ADC_Initialize()
{

  MX_GPIO_Init();
  HAL_ADC_Init(&hadc1); // the problem here is I do not know what goes on the inside of this function, but I know there should be something
  // in the brackets here
  HAL_ADC_Init(&hadc2);

  // This is probably where you run all the starter code too for the analog watchdog

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc2);

  adcInitialized = 1;

  return ADC_OK;
}

ADC_STATUS ADC_Deinitialize()
{
  if (!adcInitialized)
  {
    return ADC_ERROR_NOT_INITIALIZED;
  }

  HAL_GPIO_DeInit(ADC_1_GPIO_Port, ADC_1_Pin);
  HAL_GPIO_DeInit(ADC_2_GPIO_Port, ADC_1_Pin);
  // parameters are a problem again
  HAL_ADC_DeInit(&hadc1);
  HAL_ADC_DeInit(&hadc2);

  return ADC_OK;
}

ADC_STATUS ADC_GetValue(ADC_NUMBER num, uint16_t *pointer)
{
  if (!adcInitialized)
  {
    return ADC_ERROR_NOT_INITIALIZED;
  }

  if (num == ADC_1)
  {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1);
    *pointer = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
  }
  else if (num == ADC_2)
  {
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 1);
    *pointer = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);
  }
  else
  {
    return ADC_ERROR_INVALID_ARGUMENT;
  }

  return ADC_OK;
}

ADC_STATUS ADC_GetState(ADC_NUMBER num, ADC_STATE *state)
{
  if (!adcInitialized)
  {
    return ADC_ERROR_NOT_INITIALIZED;
  }

  uint16_t val;

  if (num == ADC_1)
  {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1);
    val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    if (val > AnalogWDGConfig_1.HighThreshold)
    {
      *state = ADC_STATE_ABOVE_WINDOW;
    }
    else if (val < AnalogWDGConfig_1.LowThreshold)
    {
      *state = ADC_STATE_BELOW_WINDOW;
    }
    else
    {
      *state = ADC_STATE_WITHIN_WINDOW;
    }
  }
  else if (num == ADC_2)
  {
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 1);
    val = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);

    if (val > AnalogWDGConfig_2.HighThreshold)
    {
      *state = ADC_STATE_ABOVE_WINDOW;
    }
    else if (val < AnalogWDGConfig_2.LowThreshold)
    {
      *state = ADC_STATE_BELOW_WINDOW;
    }
    else
    {
      *state = ADC_STATE_WITHIN_WINDOW;
    }
  }
  else
  {
    return ADC_ERROR_INVALID_ARGUMENT;
  }

  return ADC_OK;
}

ADC_STATUS ADC_ConfigureWatchdogThreshold(ADC_NUMBER num, ADC_THRESHOLD threshold, uint16_t value)
{
  if (!adcInitialized)
  {
    return ADC_ERROR_NOT_INITIALIZED;
  }

  if (num == ADC_1)
  {

    if (threshold == ADC_THRESHOLD_LOW)
    {
      AnalogWDGConfig_1.LowThreshold = value;
    }
    else if (threshold == ADC_THRESHOLD_HIGH)
    {
      AnalogWDGConfig_1.HighThreshold = value;
    }
    else
    {
      return ADC_ERROR_INVALID_ARGUMENT;
    }

    if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig_1) != HAL_OK)
    {
      Error_Handler();
    }
  }
  else if (num == ADC_2)
  {

    if (threshold == ADC_THRESHOLD_LOW)
    {
      AnalogWDGConfig_2.LowThreshold = value;
    }
    else if (threshold == ADC_THRESHOLD_HIGH)
    {
      AnalogWDGConfig_2.HighThreshold = value;
    }
    else
    {
      return ADC_ERROR_INVALID_ARGUMENT;
    }

    if (HAL_ADC_AnalogWDGConfig(&hadc2, &AnalogWDGConfig_2) != HAL_OK)
    {
      Error_Handler();
    }
  }
  else
  {
    return ADC_ERROR_INVALID_ARGUMENT;
  }

  // the idea is right here, but we are missing a parameter i believe
  // the other problem is there's nothing to set this to the lower or the upper threshold

  return ADC_OK;
}

ADC_STATUS ADC_RegisterWatchdogCallback(void (*fcn)(ADC_NUMBER *num))
{

  if (!adcInitialized)
  {
    return ADC_ERROR_NOT_INITIALIZED;
  }

  if (!fcn)
  {
    return ADC_ERROR_INVALID_ARGUMENT;
  }

  adc_callback = fcn;

  return ADC_OK;
}

ADC_STATUS ADC_ConfigureInterrupt(ADC_INTERRUPT interrupt)
{
  // Do we need an ADC Number on this?  I assume we might

  if (!adcInitialized)
  {
    return ADC_ERROR_NOT_INITIALIZED;
  }

  if (interrupt == ADC_INTERRUPT_DISABLED)
  {
    AnalogWDGConfig_1.ITMode = DISABLE;
    AnalogWDGConfig_2.ITMode = DISABLE;
  }
  else if (interrupt == ADC_INTERRUPT_ENABLED)
  {
    AnalogWDGConfig_1.ITMode = ENABLE;
    AnalogWDGConfig_2.ITMode = ENABLE;
  }
  else
  {
    return ADC_ERROR_INVALID_ARGUMENT;
  }

  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADC_AnalogWDGConfig(&hadc2, &AnalogWDGConfig_2) != HAL_OK)
  {
    Error_Handler();
  }

  return ADC_OK;
}

/* USER CODE END 1 */
