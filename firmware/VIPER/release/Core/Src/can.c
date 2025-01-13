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

#include "queue.h"
#include <string.h>
#include "enc_dec_utils.h"
#include "at24c04c.h"

VIPER_CAN_TypeDef viper_can;

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
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  if (HAL_CAN_Start(&hcan) != HAL_OK)
      {
          /* Start Error */
          Error_Handler();
      }

      viper_can.hcan = hcan;


// tell my brain this is a new copy

      // todo read id from eeprom
  //    uint8_t eeprom_buff[1];
  //    AT24C04C_ReadData(&at24c04c_1, EEPROM_ADDR_CAN_ID, eeprom_buff, sizeof(uint8_t));
  //    viper_can.id = eeprom_buff[0];
      viper_can.id = 0x14;

      viper_can.TxHeader.RTR = CAN_RTR_DATA;
      viper_can.TxHeader.IDE = CAN_ID_EXT;
      viper_can.TxHeader.DLC = 0;
      viper_can.TxHeader.TransmitGlobalTime = DISABLE;

      // filter any messages not addressed to self
      //ADD ABILITY TO READ ESTOP
      // see 24.7.4 Identifier Filtering in STM32F103C8T6 reference manual for filter configuration

      //WANT THESE TWO SPECIFIC IDS
      //0x31FF
      //0x00FF
      viper_can.canfilter_global1.FilterBank = 0;
      viper_can.canfilter_global1.FilterIdLow = ((((ESTOP_MESSAGE << CAN_MESSAGE_COMMAND_OFFSET) | (0xFF << CAN_MESSAGE_DEVICE_ID_OFFSET)) << 3) | 0b100) & 0xffff;
      viper_can.canfilter_global1.FilterIdHigh = ((((ESTOP_MESSAGE << CAN_MESSAGE_COMMAND_OFFSET) | (0xFF << CAN_MESSAGE_DEVICE_ID_OFFSET)) << 3) & 0xffff0000)
              >> 16;
      viper_can.canfilter_global1.FilterMaskIdLow = (((((DISABLE_MESSAGE << CAN_MESSAGE_COMMAND_OFFSET) | (0xFF << CAN_MESSAGE_DEVICE_ID_OFFSET)) << 3)) | 0b100) & 0xffff;
      viper_can.canfilter_global1.FilterMaskIdHigh = ((((DISABLE_MESSAGE << CAN_MESSAGE_COMMAND_OFFSET) | (0xFF << CAN_MESSAGE_DEVICE_ID_OFFSET)) << 3) & 0xffff0000)
              >> 16;
      viper_can.canfilter_global1.FilterMode = CAN_FILTERMODE_IDLIST;
      viper_can.canfilter_global1.FilterScale = CAN_FILTERSCALE_32BIT;
      viper_can.canfilter_global1.FilterFIFOAssignment = CAN_RX_FIFO0;
      viper_can.canfilter_global1.FilterActivation = ENABLE;
      viper_can.canfilter_global1.SlaveStartFilterBank = 14;

      HAL_CAN_ConfigFilter(&(viper_can.hcan), &(viper_can.canfilter_global1));

      //WANT ALL ZEROS EXCEPT COMMAND ID
      viper_can.canfilter_global2.FilterBank = 1;
      viper_can.canfilter_global2.FilterIdLow = ((CAN_MESSAGE_IDENTIFIER_GLOBAL << CAN_MESSAGE_IDENTIFIER_OFFSET) << 3) & 0xffff;
      viper_can.canfilter_global2.FilterIdHigh = (((CAN_MESSAGE_IDENTIFIER_GLOBAL << CAN_MESSAGE_IDENTIFIER_OFFSET) << 3) & 0xffff0000)
              >> 16;
      viper_can.canfilter_global2.FilterMaskIdLow = (~(CAN_MESSAGE_COMMAND_MASK << CAN_MESSAGE_COMMAND_OFFSET) << 3) & 0xffff;
      viper_can.canfilter_global2.FilterMaskIdHigh = ((~(CAN_MESSAGE_COMMAND_MASK << CAN_MESSAGE_COMMAND_OFFSET) << 3) & 0xffff0000)
              >> 16;
      viper_can.canfilter_global2.FilterMode = CAN_FILTERMODE_IDMASK;
      viper_can.canfilter_global2.FilterScale = CAN_FILTERSCALE_32BIT;
      viper_can.canfilter_global2.FilterFIFOAssignment = CAN_RX_FIFO0;
      viper_can.canfilter_global2.FilterActivation = ENABLE;
      viper_can.canfilter_global2.SlaveStartFilterBank = 14;

      HAL_CAN_ConfigFilter(&(viper_can.hcan), &(viper_can.canfilter_global2));

      viper_can.canfilterconfig.FilterBank = 2;
      viper_can.canfilterconfig.FilterIdLow = (((CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) | (viper_can.id << CAN_MESSAGE_DEVICE_ID_OFFSET)) << 3) & 0xffff;
      viper_can.canfilterconfig.FilterIdHigh = ((((CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) | (viper_can.id << CAN_MESSAGE_DEVICE_ID_OFFSET)) << 3) & 0xffff0000)
              >> 16;
      viper_can.canfilterconfig.FilterMaskIdLow = (((CAN_MESSAGE_IDENTIFIER_MASK << CAN_MESSAGE_IDENTIFIER_OFFSET) | (CAN_MESSAGE_DEVICE_ID_MASK << CAN_MESSAGE_DEVICE_ID_OFFSET)) << 3) & 0xffff;
      viper_can.canfilterconfig.FilterMaskIdHigh = ((((CAN_MESSAGE_IDENTIFIER_MASK << CAN_MESSAGE_IDENTIFIER_OFFSET) | (CAN_MESSAGE_DEVICE_ID_MASK << CAN_MESSAGE_DEVICE_ID_OFFSET)) << 3) & 0xffff0000)
              >> 16;
      viper_can.canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
      viper_can.canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
      viper_can.canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
      viper_can.canfilterconfig.FilterActivation = ENABLE;
      viper_can.canfilterconfig.SlaveStartFilterBank = 14;

      HAL_CAN_ConfigFilter(&(viper_can.hcan), &(viper_can.canfilterconfig));
      HAL_CAN_ActivateNotification(&(viper_can.hcan), CAN_IT_RX_FIFO0_MSG_PENDING); //enable interrupts

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

  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
 * @brief  Rx Fifo 0 message pending callback in non blocking mode
 * @param  CanHandle: pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *canHandle)
{
    /* Get RX message */
    if (HAL_CAN_GetRxMessage(canHandle, CAN_RX_FIFO0, &(viper_can.RxHeader),
            viper_can.RxData) != HAL_OK)
    {
        /* Reception Error */
        Error_Handler();
    }

    VIPER_CAN_Message_TypeDef *new_message = malloc(
            sizeof(VIPER_CAN_Message_TypeDef));

    if (new_message == NULL)
    {
        // todo handle error
        return;
    }

    new_message->command_id = (viper_can.RxHeader.ExtId >> CAN_MESSAGE_COMMAND_OFFSET) & (CAN_MESSAGE_COMMAND_MASK);
//    memcpy(new_message->data, viper_can.RxData, 8);
    new_message->dlc = viper_can.RxHeader.DLC;

    uint8_t *data_ptr = malloc(sizeof(uint8_t) * new_message->dlc);

    if (data_ptr == NULL)
    {
        // todo handle error
        return;
    }

    //CHECK ESTOP OR DISABLE HERE?? WRITE DIRECT TO QUEUE FRONT
    new_message->data = memcpy(data_ptr, viper_can.RxData,
            sizeof(uint8_t) * new_message->dlc);

    if (((viper_can.RxHeader.ExtId >> CAN_MESSAGE_IDENTIFIER_OFFSET) & CAN_MESSAGE_IDENTIFIER_MASK) == CAN_MESSAGE_IDENTIFIER_GLOBAL)
    {
        queue_enqueue(&can_message_queue_global, new_message);
    }
    else if (((viper_can.RxHeader.ExtId >> CAN_MESSAGE_IDENTIFIER_OFFSET) & CAN_MESSAGE_IDENTIFIER_MASK) == CAN_MESSAGE_IDENTIFIER_VIPER)
    {
        queue_enqueue(&can_message_queue_viper, new_message);
    }


}

void MX_CAN_UpdateIdAndFilters(VIPER_CAN_TypeDef *viper_can_handle)
{
	viper_can_handle->id = viper_params.VIPER_ID;

	viper_can_handle->canfilterconfig.FilterIdLow = (((CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) | (viper_can_handle->id << CAN_MESSAGE_DEVICE_ID_OFFSET)) << 3) & 0xffff;
	viper_can_handle->canfilterconfig.FilterIdHigh = ((((CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) | (viper_can_handle->id << CAN_MESSAGE_DEVICE_ID_OFFSET)) << 3) & 0xffff0000)
			>> 16;

    HAL_CAN_ConfigFilter(&(viper_can_handle->hcan), &(viper_can_handle->canfilterconfig));


}

// todo: adapt this for specific status messages for each card

//void MX_CAN_Broadcast_Odometry_Message(VIPER_CAN_TypeDef *viper_can_handle, VIPER_STATUS_TypeDef status)
//{
//    encode_double_big_endian(status.current_angle, &(viper_can_handle->TxData[0]));
//    viper_can_handle->TxHeader.DLC = sizeof(double); //double
//    viper_can_handle->TxHeader.ExtId = __encode_ext_can_id(viper_can_handle->id, SEND_ODOM_ANGLE);
//
//    HAL_CAN_AddTxMessage(&(viper_can_handle->hcan), &(viper_can_handle->TxHeader),
//            viper_can_handle->TxData, &(viper_can_handle->TxMailbox));
//}


void MX_CAN_Broadcast_Health_Message(VIPER_CAN_TypeDef *viper_can_handle, VIPER_STATUS_TypeDef status)
{
    viper_can_handle->TxData[0] = status.EEPROM_STATUS;
    viper_can_handle->TxData[1] = status.MUX_STATUS;
    // todo: fill state 2 with something (viper_can_handle->TxData[2] = status.MUX_STATUS;)
    viper_can_handle->TxData[3] = status.VIPER_STATE;
    viper_can_handle->TxData[4] = status.CARD1_STATUS;
    viper_can_handle->TxData[5] = status.CARD2_STATUS;
    viper_can_handle->TxData[6] = status.CARD3_STATUS;
    viper_can_handle->TxData[7] = status.CARD4_STATUS;

    viper_can_handle->TxHeader.DLC = 5; //float
    viper_can_handle->TxHeader.ExtId = __encode_ext_can_id(viper_can_handle->id, SEND_HEALTH_STATUS);

    HAL_CAN_AddTxMessage(&(viper_can_handle->hcan), &(viper_can_handle->TxHeader),
            viper_can_handle->TxData, &(viper_can_handle->TxMailbox));
}

void MX_CAN_Broadcast_Double_Data(VIPER_CAN_TypeDef *viper_can_handle, double value, uint16_t message_id)
{
    encode_double_big_endian(value, &(viper_can_handle->TxData[0]));
    viper_can_handle->TxHeader.DLC = sizeof(double); //float
    viper_can_handle->TxHeader.ExtId = __encode_ext_can_id(viper_can_handle->id, message_id);

    HAL_CAN_AddTxMessage(&(viper_can_handle->hcan), &(viper_can_handle->TxHeader),
            viper_can_handle->TxData, &(viper_can_handle->TxMailbox));
}

void MX_CAN_Broadcast_Uint32_Data(VIPER_CAN_TypeDef *viper_can_handle, uint32_t value, uint16_t message_id)
{
    encode_uint32_big_endian(value, &(viper_can_handle->TxData[0]));
    viper_can_handle->TxHeader.DLC = sizeof(uint32_t); //float
    viper_can_handle->TxHeader.ExtId = __encode_ext_can_id(viper_can_handle->id, message_id);

    HAL_CAN_AddTxMessage(&(viper_can_handle->hcan), &(viper_can_handle->TxHeader),
            viper_can_handle->TxData, &(viper_can_handle->TxMailbox));
}

void MX_CAN_Broadcast_Uint16_Data(VIPER_CAN_TypeDef *viper_can_handle, uint16_t value, uint16_t message_id)
{
    encode_uint32_big_endian(value, &(viper_can_handle->TxData[0]));
    viper_can_handle->TxHeader.DLC = sizeof(uint16_t); //float
    viper_can_handle->TxHeader.ExtId = __encode_ext_can_id(viper_can_handle->id, message_id);

    HAL_CAN_AddTxMessage(&(viper_can_handle->hcan), &(viper_can_handle->TxHeader),
            viper_can_handle->TxData, &(viper_can_handle->TxMailbox));
}

void MX_CAN_Broadcast_Uint8_Data(VIPER_CAN_TypeDef *viper_can_handle, uint8_t value, uint16_t message_id)
{
    viper_can_handle->TxData[0] = value;
    viper_can_handle->TxHeader.DLC = sizeof(uint8_t); //float
    viper_can_handle->TxHeader.ExtId = __encode_ext_can_id(viper_can_handle->id, message_id);

    HAL_CAN_AddTxMessage(&(viper_can_handle->hcan), &(viper_can_handle->TxHeader),
            viper_can_handle->TxData, &(viper_can_handle->TxMailbox));
}

uint32_t __encode_ext_can_id(uint8_t device_id, uint8_t message_id)
{
    // return a value that combines both the device ID and the
    // message ID so that the message can be identified
    return (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) |
            (CAN_MESSAGE_RESPONSE_VIPER << CAN_MESSAGE_RESPONSE_OFFSET) |
            (message_id << CAN_MESSAGE_COMMAND_OFFSET) | (device_id << CAN_MESSAGE_DEVICE_ID_OFFSET);
}

/* USER CODE END 1 */
