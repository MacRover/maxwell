/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
#include "i2c.h"

/* USER CODE BEGIN 0 */

uint8_t i2cInitialized;

uint8_t device_identifier = 0b10100000;
uint8_t a2_pin = 0;
uint8_t a1_pin = 0;
uint8_t read_write;
uint8_t i2c_address;

uint8_t *i2c_send_data;

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

EEPROM_STATUS EEPROM_Initialize()
{
	MX_I2C1_Init();

	i2cInitialized = 1;
	return EEPROM_OK;
}

EEPROM_STATUS EEPROM_Write(uint16_t addr, uint8_t *data, uint16_t len_bytes)
{
  if (!i2cInitialized)
	{
		return EEPROM_ERROR_NOT_INITIALIZED;
	}

  read_write = 0;
  i2c_address = device_identifier | (a2_pin << 3) | (a1_pin << 2) | ((addr & 0x1FF) >> 8) << 1 | read_write; //limit eeprom address to 9 bits, get eeprom addr MSB

  i2c_send_data = malloc((len_bytes + 1) * sizeof(uint8_t));

  i2c_send_data[0] = (uint8_t) addr & 0xff; //8 LSBs

  memcpy(&i2c_send_data[1], data, len_bytes); //copy over data to write

   while (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) i2c_address, &i2c_send_data[0], sizeof(i2c_send_data), 10000) != HAL_OK) //10000ms timeout - 10s
  {
      uint16_t I2C_Error = HAL_I2C_GetError(&hi2c1);

      if (I2C_Error == HAL_I2C_ERROR_TIMEOUT)
      {
        return EERPOM_ERROR_TIMEOUT;
      }
      else if (I2C_Error != HAL_I2C_ERROR_AF)
      {
        return EEPROM_ERROR_DATA;
      }
  }

  free(i2c_send_data);

  return EEPROM_OK;
}

EEPROM_STATUS EEPROM_Clear(uint16_t addr, uint16_t len_bytes)
{
  if (!i2cInitialized)
	{
		return EEPROM_ERROR_NOT_INITIALIZED;
	}

  read_write = 0;
  i2c_address = device_identifier | (a2_pin << 3) | (a1_pin << 2) | ((addr & 0x1FF) >> 8) << 1 | read_write; //limit eeprom address to 9 bits, get eeprom addr MSB

  i2c_send_data = calloc((len_bytes + 1), sizeof(uint8_t));

  i2c_send_data[0] = (uint8_t) addr & 0xff; //8 LSBs

   while (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) i2c_address, &i2c_send_data[0], sizeof(i2c_send_data), 10000) != HAL_OK) //10000ms timeout - 10s
  {
      uint16_t I2C_Error = HAL_I2C_GetError(&hi2c1);

      if (I2C_Error == HAL_I2C_ERROR_TIMEOUT)
      {
        return EERPOM_ERROR_TIMEOUT;
      }
      else if (I2C_Error != HAL_I2C_ERROR_AF)
      {
        return EEPROM_ERROR_DATA;
      }
  }

  free(i2c_send_data);

  return EEPROM_OK;
}

EEPROM_STATUS EEPROM_Read(uint16_t addr, uint8_t *data, uint16_t len_bytes)
{

  if (!i2cInitialized)
	{
		return EEPROM_ERROR_NOT_INITIALIZED;
	}

  read_write = 1;
  i2c_address = device_identifier | (a2_pin << 3) | (a1_pin << 2) | ((addr & 0x1FF) >> 8) << 1 | read_write; // limit eeprom address to 9 bits, get eeprom addr MSB

  while (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(i2c_address & 0xFE), (uint16_t) (addr & 0xFF), I2C_MEMADD_SIZE_8BIT, data, len_bytes, 10000) != HAL_OK)
  {

    uint16_t I2C_Error = HAL_I2C_GetError(&hi2c1);

    if (I2C_Error == HAL_I2C_ERROR_TIMEOUT)
    {
      return EERPOM_ERROR_TIMEOUT;
    }
    else if (I2C_Error != HAL_I2C_ERROR_AF)
    {
      return EEPROM_ERROR_DATA;
    }
  }

  return EEPROM_OK;
}

EEPROM_STATUS EEPROM_Deinitialize()
{
	if (!i2cInitialized)
	{
		return EEPROM_ERROR_NOT_INITIALIZED;
	}

	i2cInitialized=0;
	if (HAL_I2C_DeInit(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

	return EEPROM_OK;

}
/* USER CODE END 1 */
