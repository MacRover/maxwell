/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
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
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi1;

extern SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_SPI1_Init(void);
void MX_SPI2_Init(void);

/* USER CODE BEGIN Prototypes */

typedef enum 
{
  STEPPER_OK = 0,
  STEPPER_ERROR_NOT_INITIALIZED,
  STEPPER_ERROR_HAL,
  STEPPER_ERROR_INVALID_ARGUMENT
} STEPPER_STATUS;

typedef enum 
{
  STEPPER_DIRECTION_CW = 0,
  STEPPER_DIRECTION_CCW
} STEPPER_DIRECTION;

typedef enum 
{
  STEPPER_REGISTER_DRVCONF,
  STEPPER_REGISTER_SGCONF,
  STEPPER_REGISTER_SMARTEN,
  STEPPER_REGISTER_CHOPCONF, 
  STEPPER_REGISTER_DRVCTRL,
  STEPPER_REGISTER_ALL
} STEPPER_REGISTER;

typedef struct __attribute__((__packed__))
{
  uint8_t TST     : 1; 
  uint8_t SLP     : 5;
  uint8_t DIS_S2G : 1; 
  uint8_t TS2G    : 2;
  uint8_t SDOFF   : 1;
  uint8_t VSENSE  : 1;
  uint8_t RDSEL   : 2;
  uint8_t OTSENS  : 1;
  uint8_t SHRTSENS : 1;
  uint8_t EN_PFD  : 1;
  uint8_t EN_S2VS : 1;
} STEPPER_DRVCONF;

typedef struct __attribute__((__packed__))
{
  uint8_t SFILT : 1;
  uint8_t SGT   : 7;
  uint8_t CS    : 8;
} STEPPER_SGCONF;

typedef struct __attribute__((__packed__))
{
   uint8_t SEIMIN : 1;
   uint16_t SEDN  : 13;
   uint8_t SEMAX  : 4;
   uint8_t SEUP   : 2;
   uint8_t SEMIN  : 4;
} STEPPER_SMARTEN;

typedef struct __attribute__((__packed__))
{
  uint8_t TBL   : 2;
  uint8_t CHM   : 1;
  uint8_t RNDTF : 1;
  uint8_t HDEC  : 2;
  uint8_t HEND  : 4;
  uint8_t HSTRT : 3;
  uint8_t TOFF  : 4; 
} STEPPER_CHOPCONF;

typedef struct __attribute__((__packed__))
{
  uint8_t INTPOL  : 1;
  uint8_t DEDGE   : 1;
  uint8_t  MRES    : 4; 
} STEPPER_DRVCTRL;

typedef union
{
  //use value for reading/writing to EEPROM only
  uint32_t value[3]; //96 bits, 12 bytes
  struct __attribute__((__packed__))
  {
    STEPPER_DRVCONF drvconf;
    STEPPER_SGCONF sgconf;
    STEPPER_SMARTEN smarten;
    STEPPER_CHOPCONF chopconf;
    STEPPER_DRVCTRL drvctrl;
  } reg;

} STEPPER_REGISTER_DATA;


STEPPER_STATUS STEPPER_Initialize(void);
STEPPER_STATUS STEPPER_WriteRegisterConfig(STEPPER_REGISTER reg, STEPPER_REGISTER_DATA *data);
STEPPER_STATUS STEPPER_ReadRegisterConfig(STEPPER_REGISTER reg, STEPPER_REGISTER_DATA *data);
STEPPER_STATUS STEPPER_ReadRegisterResponse(uint32_t *rsp);
STEPPER_STATUS STEPPER_StartStep(void);
STEPPER_STATUS STEPPER_StopStep(void);
STEPPER_STATUS STEPPER_AdjustStepSpeed(void); //TBD
STEPPER_STATUS STEPPER_SetDirection(STEPPER_DIRECTION dir);
STEPPER_STATUS STEPPER_Deinitialize(void);


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

