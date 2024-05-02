/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
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
#include "spi.h"

/* USER CODE BEGIN 0 */
void _prepareDriverTransmit(STEPPER_REGISTER reg, STEPPER_REGISTER_DATA *data);
HAL_StatusTypeDef _transmitReceiveDriver_3Bytes(void);

uint8_t driverInitialized;

uint32_t driverTransmit;
uint32_t driverReceive;

STEPPER_REGISTER_DATA localData;
/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}
/* SPI2 init function */
void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = DRIVER_SCK_Pin|DRIVER_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DRIVER_MISO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DRIVER_MISO_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
  else if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* SPI2 clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, DRIVER_SCK_Pin|DRIVER_MISO_Pin|DRIVER_MOSI_Pin);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
  else if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

// STEPPER LIBRARY

void _prepareDriverTransmit(STEPPER_REGISTER reg, STEPPER_REGISTER_DATA *data)
{

  // Bits 19, 18, 17
  // 111 DRVCONF (0b111 << 17)
  // 110 SGCSCONF (0b110 << 17)
  // 101 SMARTEN (0b101 << 17)
  // 100 CHOPCONF (0b100 << 17)
  // 00X DRVCTRL (0b00 << 18)

  switch (reg)
  {
    case STEPPER_REGISTER_DRVCONF:
    {
      STEPPER_DRVCONF reg = data->reg.drvconf;
      driverTransmit = 0;
      driverTransmit |= (0b111 << 17); // DRVCONF
      driverTransmit |= (reg.TST << 16); // TST: test mode
      driverTransmit |= (reg.SLP << 11); // SLP: Slope control
      driverTransmit |= (reg.DIS_S2G << 10); // DIS_S2G: Short to ground protection
      driverTransmit |= (reg.TS2G << 8); // TS2G: Short detection delay
      driverTransmit |= (reg.SDOFF << 7); // SDOFF: step/dir interface
      driverTransmit |= (reg.VSENSE << 6); // VSENSE: full-scale sense resistor voltage setting
      driverTransmit |= (reg.RDSEL << 4); // RDSEL: read out select
      driverTransmit |= (reg.OTSENS << 3); // OTSENS: overtemp shutdown setting
      driverTransmit |= (reg.SHRTSENS << 2); // SHRTSENS: short to ground sensitivity
      driverTransmit |= (reg.EN_PFD << 1); // EN_PFD: passive fast delay setting
      driverTransmit |= (reg.EN_S2VS << 0); // EN_S2VS: Short to VS protection

      break;
    }
    case STEPPER_REGISTER_SGCONF:
    {
      STEPPER_SGCONF reg = data->reg.sgconf;
      driverTransmit = 0;
      driverTransmit |= (0b110 << 17); // SGCSCONF
      driverTransmit |= (reg.SFILT << 16); // SFILT: stall guard filter
      driverTransmit |= (reg.SGT << 8); // SGT: stall guard threshold
      driverTransmit |= (reg.CS << 0); // CS: current scale

      break;
    }
    case STEPPER_REGISTER_SMARTEN:
    {
      STEPPER_SMARTEN reg = data->reg.smarten;
      driverTransmit = 0;
      driverTransmit |= (0b101 << 17); // SMARTEN
      driverTransmit |= (reg.SEIMIN << 15); // SEIMIN: min cool step current
      driverTransmit |= (reg.SEDN << 13); // SEDN: current dec. speed
      driverTransmit |= (reg.SEMAX << 8); // SEMAX: upper cool step threshold offset
      driverTransmit |= (reg.SEUP << 5); // SEUP: current increment size
      driverTransmit |= (reg.SEMIN << 0); // SEMIN: cool step lower threshold

      break;
    }
    case STEPPER_REGISTER_CHOPCONF:
    {
      STEPPER_CHOPCONF reg = data->reg.chopconf;
      driverTransmit = 0;
      driverTransmit |= (0b100 << 17); // CHOPCONF
      driverTransmit |= (reg.TBL << 15); // TBL: blanking time
      driverTransmit |= (reg.CHM << 14); // CHM: chopper mode
      driverTransmit |= (reg.RNDTF << 13); // RNDTF: Random TOFF time
      driverTransmit |= (reg.HDEC << 11); // HDEC: hysteresis decay or fast decay mode
      driverTransmit |= (reg.HEND << 7); // HEND: hysteresis end value
      driverTransmit |= (reg.HSTRT << 4); // HSTRT: hysteresis start value
      driverTransmit |= (reg.TOFF << 0); // TOFF: mosfet off time
    }
    case STEPPER_REGISTER_DRVCTRL:
    {
      STEPPER_DRVCTRL reg = data->reg.drvctrl;
      driverTransmit = 0;
      driverTransmit |= (0b00 << 18); // DRVCTRL
      driverTransmit |= (reg.INTPOL << 9); // INTPOL: step interpolation
      driverTransmit |= (reg.DEDGE << 8); // DEDGE: Double edge step pulses
      driverTransmit |= (reg.MRES << 0); // MRES: Microsteps per fullstep

      break;
    }

    default:
      break;
  }
}

HAL_StatusTypeDef _transmitReceiveDriver_3Bytes()
{
    uint8_t SPImsg_bytes[3];
    uint8_t SPIread_bytes[3];

    SPImsg_bytes[2] = (uint8_t) (driverTransmit & 0xFF);
    SPImsg_bytes[1] = (uint8_t) ((driverTransmit & 0xFF00) >> 8);
    SPImsg_bytes[0] = (uint8_t) ((driverTransmit & 0xFF0000) >> 16);

    HAL_GPIO_WritePin(DRIVER_CS_GPIO_Port, DRIVER_CS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef spi_status = HAL_SPI_TransmitReceive(&hspi1, SPImsg_bytes, SPIread_bytes, 3, 1000);
    HAL_GPIO_WritePin(DRIVER_CS_GPIO_Port, DRIVER_CS_Pin, GPIO_PIN_SET);

    driverReceive = (SPIread_bytes[0] << 16) | (SPIread_bytes[1] << 8) | SPIread_bytes[2];

    return spi_status;
}

STEPPER_STATUS STEPPER_Initialize()
{
  MX_SPI1_Init();
  MX_GPIO_Init(); 

  driverInitialized = 1;

  return STEPPER_OK;
}

STEPPER_STATUS STEPPER_WriteRegisterConfig(STEPPER_REGISTER reg, STEPPER_REGISTER_DATA *data)
{
  if (!driverInitialized)
  {
    return STEPPER_ERROR_NOT_INITIALIZED;
  }

  if (!data)
  {
    return STEPPER_ERROR_INVALID_ARGUMENT;
  }

  if ((reg == STEPPER_REGISTER_DRVCONF) || (reg == STEPPER_REGISTER_ALL))
  {

    localData.reg.drvconf = data->reg.drvconf;
    _prepareDriverTransmit(STEPPER_REGISTER_DRVCONF, data);
    if (_transmitReceiveDriver_3Bytes() != HAL_OK)
    {
      return STEPPER_ERROR_HAL;
    }

  }

  if ((reg == STEPPER_REGISTER_SGCONF) || (reg == STEPPER_REGISTER_ALL))
  {
    
    localData.reg.sgconf = data->reg.sgconf;
    _prepareDriverTransmit(STEPPER_REGISTER_SGCONF, data);
    if (_transmitReceiveDriver_3Bytes() != HAL_OK)
    {
      return STEPPER_ERROR_HAL;
    }
  }

  if ((reg == STEPPER_REGISTER_SMARTEN) || (reg == STEPPER_REGISTER_ALL))
  {
    
    localData.reg.smarten = data->reg.smarten;
    _prepareDriverTransmit(STEPPER_REGISTER_SMARTEN, data);
    if (_transmitReceiveDriver_3Bytes() != HAL_OK)
    {
      return STEPPER_ERROR_HAL;
    }

  }
  
  if ((reg == STEPPER_REGISTER_CHOPCONF) || (reg == STEPPER_REGISTER_ALL))
  {
   
    localData.reg.chopconf = data->reg.chopconf;
    _prepareDriverTransmit(STEPPER_REGISTER_CHOPCONF, data);
    if (_transmitReceiveDriver_3Bytes() != HAL_OK)
    {
      return STEPPER_ERROR_HAL;
    }

  }

  if ((reg == STEPPER_REGISTER_DRVCTRL) || (reg == STEPPER_REGISTER_ALL))
  {
   
    localData.reg.drvctrl = data->reg.drvctrl;
    _prepareDriverTransmit(STEPPER_REGISTER_DRVCTRL, data);
    if (_transmitReceiveDriver_3Bytes() != HAL_OK)
    {
      return STEPPER_ERROR_HAL;
    }

  }
  else
  {
    return STEPPER_ERROR_INVALID_ARGUMENT;
  }
  
  return STEPPER_OK;
}

STEPPER_STATUS STEPPER_ReadRegisterConfig(STEPPER_REGISTER reg, STEPPER_REGISTER_DATA *data)
{

  if (!driverInitialized)
  {
    return STEPPER_ERROR_NOT_INITIALIZED;
  }

  if (!data)
  {
    return STEPPER_ERROR_INVALID_ARGUMENT;
  }

  switch(reg)
  {
    case STEPPER_REGISTER_DRVCONF:
      memcpy(&(data->reg.drvconf), &(localData.reg.drvconf), sizeof(STEPPER_DRVCONF));
      break;

    case STEPPER_REGISTER_SGCONF:
      memcpy(&(data->reg.sgconf), &(localData.reg.sgconf), sizeof(STEPPER_SGCONF));
      break;

    case STEPPER_REGISTER_SMARTEN:
      memcpy(&(data->reg.smarten), &(localData.reg.smarten), sizeof(STEPPER_SMARTEN));
      break;

    case STEPPER_REGISTER_CHOPCONF:
      memcpy(&(data->reg.chopconf), &(localData.reg.chopconf), sizeof(STEPPER_CHOPCONF));
      break;

    case STEPPER_REGISTER_DRVCTRL:
      memcpy(&(data->reg.drvctrl), &(localData.reg.drvctrl), sizeof(STEPPER_DRVCTRL));
      break;
    
    case STEPPER_REGISTER_ALL:
      memcpy(data, &(localData), sizeof(STEPPER_REGISTER_DATA));
      break;

    default:
      return STEPPER_ERROR_INVALID_ARGUMENT;
  }

  return STEPPER_OK;
}

STEPPER_STATUS STEPPER_ReadRegisterResponse(uint32_t *rsp)
{
  if (!driverInitialized)
  {
    return STEPPER_ERROR_NOT_INITIALIZED;
  }

  if (!rsp)
  {
    return STEPPER_ERROR_INVALID_ARGUMENT;
  }

  //send local copy of data, will not overwrite anything
  _prepareDriverTransmit(STEPPER_REGISTER_DRVCONF, &localData);
  if (_transmitReceiveDriver_3Bytes() != HAL_OK)
  {
    return STEPPER_ERROR_HAL;
  }

  memcpy(rsp, &driverReceive, sizeof(driverReceive));

  return STEPPER_OK;
}


STEPPER_STATUS STEPPER_StartStep(void);
STEPPER_STATUS STEPPER_StopStep(void);
STEPPER_STATUS STEPPER_AdjustStepSpeed(void); //TBD

STEPPER_STATUS STEPPER_SetDirection(STEPPER_DIRECTION dir)
{

  if (!driverInitialized)
  {
    return STEPPER_ERROR_NOT_INITIALIZED;
  }

  switch (dir)
  {
    case STEPPER_DIRECTION_CW:
      HAL_GPIO_WritePin(DRIVER_STEP_GPIO_Port, DRIVER_STEP_Pin, GPIO_PIN_RESET);
      break;

    case STEPPER_DIRECTION_CCW:
      HAL_GPIO_WritePin(DRIVER_STEP_GPIO_Port, DRIVER_STEP_Pin, GPIO_PIN_SET);
      break;

    default:
      return STEPPER_ERROR_INVALID_ARGUMENT;
  }

  return HAL_OK;
}

STEPPER_STATUS STEPPER_Deinitialize()
{
  if (!driverInitialized)
  {
    return STEPPER_ERROR_NOT_INITIALIZED;
  }

  driverInitialized = 0;
  if (HAL_SPI_DeInit(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_GPIO_DeInit(DRIVER_CS_GPIO_Port, DRIVER_CS_Pin);

  return STEPPER_OK;
}




//ENCODER LIBRARY


/* USER CODE END 1 */
