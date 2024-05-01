/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
void _setupTxCAN(void);
void _setupRxCAN(void);
void _setupRxFilter(void);
HAL_StatusTypeDef _txCAN(void);


uint8_t initialized;

CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;

// CAN Rx
CAN_FilterTypeDef canfilterconfig;
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];


void (*callback)(CAN_MESSAGE *msg) = NULL;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
    /* Get RX message */
    if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &RxHeader, RxData)
            != HAL_OK)
    {
        /* Reception Error */
        Error_Handler();
    }


    static CAN_MESSAGE rec;
    memcpy(&(rec.header), &(RxHeader.ExtId), sizeof(RxHeader.ExtId));
    memcpy(rec.data, RxData, 8*sizeof(uint8_t));

    if(callback)
    {
    	(*callback)(&rec);
    }
}


void _setupTxCAN()
{
    TxHeader.StdId = 0x321;
    TxHeader.ExtId = 0x01;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;

    TxData[0] = 0xA0;
    TxData[1] = 0xA1;
    TxData[2] = 0xA2;
    TxData[3] = 0xA3;
    TxData[4] = 0xA4;
    TxData[5] = 0xA5;
    TxData[6] = 0xA6;
    TxData[7] = 0xA7;
}

void _setupRxCAN()
{

    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); //enable interrupts
}

void _setupRxFilter()
{
	canfilterconfig.FilterBank = 0;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.FilterIdHigh = 0x0000;
	canfilterconfig.FilterIdLow = 0x0000;
	canfilterconfig.FilterMaskIdHigh = 0x0000;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilterconfig.FilterActivation = ENABLE;
	canfilterconfig.SlaveStartFilterBank = 14;

	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
}

HAL_StatusTypeDef _txCAN()
{
    HAL_StatusTypeDef can_result = HAL_CAN_AddTxMessage(&hcan, &TxHeader,
            TxData, &TxMailbox);

    return can_result;
}


//// PUBLIC FUNCTIONS BELOW


CAN_STATUS CAN_Initialize()
{
	MX_CAN_Init();
	_setupTxCAN();
	_setupRxCAN();
	_setupRxFilter();

	initialized = 1;
	return CAN_OK;

}

CAN_STATUS CAN_ConfigFilterId(uint8_t id)
{

	return CAN_OK;

}

CAN_STATUS CAN_ResetFilter()
{

	if(!initialized)
	{
		return CAN_ERROR_NOT_INITIALIZED;
	}

	_setupRxFilter();
	return CAN_OK;
}

CAN_STATUS CAN_Send(CAN_MESSAGE *msg)
{

	if(!initialized)
	{
		return CAN_ERROR_NOT_INITIALIZED;
	}

	memcpy(&(TxHeader.ExtId), &(msg->header), sizeof(TxHeader.ExtId));
	memcpy(TxData, &(msg->data), 8*sizeof(uint8_t));

	if(_txCAN() != HAL_OK)
	{
		return CAN_ERROR_DATA;
	}

	return CAN_OK;


}

CAN_STATUS CAN_RegisterReceiveCallback(void (*fcn)(CAN_MESSAGE *msg))
{
	if (!fcn)
	{
		return CAN_ERROR_INVALID_ARGUMENT;
	}

	callback = fcn;

	return CAN_OK;
}


CAN_STATUS CAN_Deinitialize()
{
	if (!initialized)
	{
		return CAN_ERROR_NOT_INITIALIZED;
	}


	initialized=0;
	if (HAL_CAN_DeInit(&hcan) != HAL_OK)
  {
  Error_Handler();
  }

	return CAN_OK;

}

/* USER CODE END 1 */
