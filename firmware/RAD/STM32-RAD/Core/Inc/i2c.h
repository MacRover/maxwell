/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
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
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */

typedef enum 
{
  EEPROM_OK = 0,
  EEPROM_ERROR_NOT_INITIALIZED,
  EEPROM_ERROR_DATA,
  EERPOM_ERROR_TIMEOUT,
  EEPROM_ERROR_FULL
} EEPROM_STATUS;

EEPROM_STATUS EEPROM_Initialize(void);
EEPROM_STATUS EEPROM_Write(uint16_t addr, uint8_t *data, uint16_t len_bytes);
EEPROM_STATUS EEPROM_Read(uint16_t addr, uint8_t *buffer, uint16_t len_bytes);
EEPROM_STATUS EEPROM_Clear(uint16_t addr, uint16_t len_bytes);
EEPROM_STATUS EEPROM_Deinitialize(void);


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

