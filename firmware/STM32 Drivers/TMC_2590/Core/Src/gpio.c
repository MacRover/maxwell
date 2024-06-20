/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

void (*callback_pressed)(LS_NUMBER *num) = NULL;
void (*callback_released)(LS_NUMBER *num) = NULL;

uint8_t initialized_limitswitch;


/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through
        * the Code Generation settings)
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DRIVER_CS_Pin|DRIVER_ENN_Pin|DRIVER_ST_ALONE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_RED_Pin|ENCODER_CS_Pin|DRIVER_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = LS_1_Pin|LS_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin */
  GPIO_InitStruct.Pin = DRIVER_CS_Pin|DRIVER_ENN_Pin|DRIVER_ST_ALONE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11
                           PB5 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin */
  GPIO_InitStruct.Pin = LED_RED_Pin|ENCODER_CS_Pin|DRIVER_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = DRIVER_SG_TEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DRIVER_SG_TEST_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 2 */

// PUBLIC FUNCTIONS --------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

  static LS_NUMBER pin;
  pin = (GPIO_Pin == LS_1_Pin) ? LS_1 : LS_2;

  if (HAL_GPIO_ReadPin(LS_1_GPIO_Port, GPIO_Pin) == GPIO_PIN_SET)
  {
    (*callback_pressed)(&pin);
  }
  else
  {
    (*callback_released)(&pin);
  }
}


LS_STATUS LS_Initialize() {

	//MX_GPIO_Init();

	initialized_limitswitch = 1;

	return LS_OK;
}

LS_STATUS LS_Deinitialize() {

	if (!initialized_limitswitch) {
		return LS_ERROR_NOT_INITIALIZED;
	}

	//HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);
	//HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);

	initialized_limitswitch = 0;
	return LS_OK;

}


LS_STATUS LS_GetState(LS_NUMBER num, LS_STATE *state) {

	if (!initialized_limitswitch) {
		return LS_ERROR_NOT_INITIALIZED;
	}

	uint8_t pin_state;


	if (num == LS_1) {
		pin_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	} else if (num == LS_2) {
		pin_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
	} else {
		return LS_ERROR_PIN;
	}

	if (pin_state == GPIO_PIN_RESET) {
		*state = LS_STATE_RELEASED;
	} else if (pin_state == GPIO_PIN_SET) {
		*state = LS_STATE_PRESSED;
	} else {
		return LS_ERROR_HAL;
	}

	return LS_OK;

}

LS_STATUS LS_RegisterPressedCallback(void (*fcn)(LS_NUMBER *num)) {

	if (!initialized_limitswitch) {
		return LS_ERROR_NOT_INITIALIZED;
	}

	if (!fcn)
	{
		return LS_ERROR_INVALID_ARGUMENT;
	}

	callback_pressed = fcn;

	return LS_OK;

}


LS_STATUS LS_RegisterReleasedCallback(void (*fcn)(LS_NUMBER *num)) {

	if (!initialized_limitswitch) {
		return LS_ERROR_NOT_INITIALIZED;
	}

	if (!fcn)
	{
		return LS_ERROR_INVALID_ARGUMENT;
	}

	callback_released = fcn;

	return LS_OK;

}


/* USER CODE END 2 */
