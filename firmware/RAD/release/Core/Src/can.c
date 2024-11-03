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

RAD_CAN_TypeDef rad_can;
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
    hcan.Init.Prescaler = 4;
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

    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        /* Start Error */
        Error_Handler();
    }

    rad_can.hcan = hcan;

    // todo read id from eeprom
//    uint8_t eeprom_buff[1];
//    AT24C04C_ReadData(&at24c04c_1, EEPROM_ADDR_CAN_ID, eeprom_buff, sizeof(uint8_t));
//    rad_can.id = eeprom_buff[0];
    rad_can.id = 0x14;

    rad_can.TxHeader.RTR = CAN_RTR_DATA;
    rad_can.TxHeader.IDE = CAN_ID_EXT;
    rad_can.TxHeader.DLC = 0;
    rad_can.TxHeader.TransmitGlobalTime = DISABLE;

    // filter any messages not addressed to self
    //ADD ABILITY TO READ ESTOP
    // see 24.7.4 Identifier Filtering in STM32F103C8T6 reference manual for filter configuration

    //WANT THESE TWO SPECIFIC IDS
    //0x31FF
    //0x00FF
    rad_can.canfilter_global1.FilterBank = 0;
    rad_can.canfilter_global1.FilterIdLow = ((((ESTOP_MESSAGE << CAN_MESSAGE_COMMAND_OFFSET) | (0xFF << CAN_MESSAGE_DEVICE_ID_OFFSET)) << 3) | 0b100) & 0xffff;
    rad_can.canfilter_global1.FilterIdHigh = ((((ESTOP_MESSAGE << CAN_MESSAGE_COMMAND_OFFSET) | (0xFF << CAN_MESSAGE_DEVICE_ID_OFFSET)) << 3) & 0xffff0000)
            >> 16;
    rad_can.canfilter_global1.FilterMaskIdLow = (((((DISABLE_MESSAGE << CAN_MESSAGE_COMMAND_OFFSET) | (0xFF << CAN_MESSAGE_DEVICE_ID_OFFSET)) << 3)) | 0b100) & 0xffff;
    rad_can.canfilter_global1.FilterMaskIdHigh = ((((DISABLE_MESSAGE << CAN_MESSAGE_COMMAND_OFFSET) | (0xFF << CAN_MESSAGE_DEVICE_ID_OFFSET)) << 3) & 0xffff0000)
            >> 16;
    rad_can.canfilter_global1.FilterMode = CAN_FILTERMODE_IDLIST;
    rad_can.canfilter_global1.FilterScale = CAN_FILTERSCALE_32BIT;
    rad_can.canfilter_global1.FilterFIFOAssignment = CAN_RX_FIFO0;
    rad_can.canfilter_global1.FilterActivation = ENABLE;
    rad_can.canfilter_global1.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&(rad_can.hcan), &(rad_can.canfilter_global1));

    //WANT ALL ZEROS EXCEPT COMMAND ID
    rad_can.canfilter_global2.FilterBank = 1;
    rad_can.canfilter_global2.FilterIdLow = ((CAN_MESSAGE_IDENTIFIER_GLOBAL << CAN_MESSAGE_IDENTIFIER_OFFSET) << 3) & 0xffff;
    rad_can.canfilter_global2.FilterIdHigh = (((CAN_MESSAGE_IDENTIFIER_GLOBAL << CAN_MESSAGE_IDENTIFIER_OFFSET) << 3) & 0xffff0000)
            >> 16;
    rad_can.canfilter_global2.FilterMaskIdLow = (~(CAN_MESSAGE_COMMAND_MASK << CAN_MESSAGE_COMMAND_OFFSET) << 3) & 0xffff;
    rad_can.canfilter_global2.FilterMaskIdHigh = ((~(CAN_MESSAGE_COMMAND_MASK << CAN_MESSAGE_COMMAND_OFFSET) << 3) & 0xffff0000)
            >> 16;
    rad_can.canfilter_global2.FilterMode = CAN_FILTERMODE_IDMASK;
    rad_can.canfilter_global2.FilterScale = CAN_FILTERSCALE_32BIT;
    rad_can.canfilter_global2.FilterFIFOAssignment = CAN_RX_FIFO0;
    rad_can.canfilter_global2.FilterActivation = ENABLE;
    rad_can.canfilter_global2.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&(rad_can.hcan), &(rad_can.canfilter_global2));

    rad_can.canfilterconfig.FilterBank = 2;
    rad_can.canfilterconfig.FilterIdLow = (((CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | (rad_can.id << CAN_MESSAGE_DEVICE_ID_OFFSET)) << 3) & 0xffff;
    rad_can.canfilterconfig.FilterIdHigh = ((((CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | (rad_can.id << CAN_MESSAGE_DEVICE_ID_OFFSET)) << 3) & 0xffff0000)
            >> 16;
    rad_can.canfilterconfig.FilterMaskIdLow = (((CAN_MESSAGE_IDENTIFIER_MASK << CAN_MESSAGE_IDENTIFIER_OFFSET) | (CAN_MESSAGE_DEVICE_ID_MASK << CAN_MESSAGE_DEVICE_ID_OFFSET)) << 3) & 0xffff;
    rad_can.canfilterconfig.FilterMaskIdHigh = ((((CAN_MESSAGE_IDENTIFIER_MASK << CAN_MESSAGE_IDENTIFIER_OFFSET) | (CAN_MESSAGE_DEVICE_ID_MASK << CAN_MESSAGE_DEVICE_ID_OFFSET)) << 3) & 0xffff0000)
            >> 16;
    rad_can.canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    rad_can.canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    rad_can.canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    rad_can.canfilterconfig.FilterActivation = ENABLE;
    rad_can.canfilterconfig.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&(rad_can.hcan), &(rad_can.canfilterconfig));
    HAL_CAN_ActivateNotification(&(rad_can.hcan), CAN_IT_RX_FIFO0_MSG_PENDING); //enable interrupts
    /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct =
    { 0 };
    if (canHandle->Instance == CAN1)
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
        HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
        /* USER CODE BEGIN CAN1_MspInit 1 */

        /* USER CODE END CAN1_MspInit 1 */
    }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *canHandle)
{

    if (canHandle->Instance == CAN1)
    {
        /* USER CODE BEGIN CAN1_MspDeInit 0 */

        /* USER CODE END CAN1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_CAN1_CLK_DISABLE();

        /**CAN GPIO Configuration
         PA11     ------> CAN_RX
         PA12     ------> CAN_TX
         */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

        /* CAN1 interrupt Deinit */
        HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
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
    if (HAL_CAN_GetRxMessage(canHandle, CAN_RX_FIFO0, &(rad_can.RxHeader),
            rad_can.RxData) != HAL_OK)
    {
        /* Reception Error */
        Error_Handler();
    }

    RAD_CAN_Message_TypeDef *new_message = malloc(
            sizeof(RAD_CAN_Message_TypeDef));

    if (new_message == NULL)
    {
        // todo handle error
        return;
    }

    new_message->command_id = (rad_can.RxHeader.ExtId >> CAN_MESSAGE_COMMAND_OFFSET) & (CAN_MESSAGE_COMMAND_MASK);
//    memcpy(new_message->data, rad_can.RxData, 8);
    new_message->dlc = rad_can.RxHeader.DLC;

    uint8_t *data_ptr = malloc(sizeof(uint8_t) * new_message->dlc);

    if (data_ptr == NULL)
    {
        // todo handle error
        return;
    }
    
    //CHECK ESTOP OR DISABLE HERE?? WRITE DIRECT TO QUEUE FRONT
    new_message->data = memcpy(data_ptr, rad_can.RxData,
            sizeof(uint8_t) * new_message->dlc);

    if (((rad_can.RxHeader.ExtId >> CAN_MESSAGE_IDENTIFIER_OFFSET) & CAN_MESSAGE_IDENTIFIER_MASK) == CAN_MESSAGE_IDENTIFIER_GLOBAL)
    {
        queue_enqueue(&can_message_queue_global, new_message);
    }
    else if (((rad_can.RxHeader.ExtId >> CAN_MESSAGE_IDENTIFIER_OFFSET) & CAN_MESSAGE_IDENTIFIER_MASK) == CAN_MESSAGE_IDENTIFIER_RAD)
    {
        queue_enqueue(&can_message_queue_rad, new_message);
    }


}

void MX_CAN_UpdateIdAndFilters(RAD_CAN_TypeDef *rad_can_handle)
{
	rad_can_handle->id = rad_params.RAD_ID;

	rad_can_handle->canfilterconfig.FilterIdLow = (((CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | (rad_can_handle->id << CAN_MESSAGE_DEVICE_ID_OFFSET)) << 3) & 0xffff;
	rad_can_handle->canfilterconfig.FilterIdHigh = ((((CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | (rad_can_handle->id << CAN_MESSAGE_DEVICE_ID_OFFSET)) << 3) & 0xffff0000)
			>> 16;

    HAL_CAN_ConfigFilter(&(rad_can_handle->hcan), &(rad_can_handle->canfilterconfig));


}

void MX_CAN_Broadcast_Odometry_Message(RAD_CAN_TypeDef *rad_can_handle, RAD_STATUS_TypeDef status)
{
    encode_double_big_endian(status.current_angle, &(rad_can_handle->TxData[0]));
    rad_can_handle->TxHeader.DLC = sizeof(double); //double
    rad_can_handle->TxHeader.ExtId = __encode_ext_can_id(rad_can_handle->id, SEND_ODOM_ANGLE);

    HAL_CAN_AddTxMessage(&(rad_can_handle->hcan), &(rad_can_handle->TxHeader),
            rad_can_handle->TxData, &(rad_can_handle->TxMailbox));
}


void MX_CAN_Broadcast_Health_Message(RAD_CAN_TypeDef *rad_can_handle, RAD_STATUS_TypeDef status)
{
    rad_can_handle->TxData[0] = status.EEPROM_STATUS;
    rad_can_handle->TxData[1] = status.TMC_STATUS;
    rad_can_handle->TxData[2] = status.ENCODER_STATUS;
    rad_can_handle->TxData[3] = status.RAD_STATE;
    rad_can_handle->TxData[4] = status.ls_1;


    rad_can_handle->TxHeader.DLC = 5; //float
    rad_can_handle->TxHeader.ExtId = __encode_ext_can_id(rad_can_handle->id, SEND_HEALTH_STATUS);

    HAL_CAN_AddTxMessage(&(rad_can_handle->hcan), &(rad_can_handle->TxHeader),
            rad_can_handle->TxData, &(rad_can_handle->TxMailbox));
}

void MX_CAN_Broadcast_Double_Data(RAD_CAN_TypeDef *rad_can_handle, double value, uint16_t message_id)
{
    encode_double_big_endian(value, &(rad_can_handle->TxData[0]));
    rad_can_handle->TxHeader.DLC = sizeof(double); //float
    rad_can_handle->TxHeader.ExtId = __encode_ext_can_id(rad_can_handle->id, message_id);

    HAL_CAN_AddTxMessage(&(rad_can_handle->hcan), &(rad_can_handle->TxHeader),
            rad_can_handle->TxData, &(rad_can_handle->TxMailbox));
}

void MX_CAN_Broadcast_Uint32_Data(RAD_CAN_TypeDef *rad_can_handle, uint32_t value, uint16_t message_id)
{
    encode_uint32_big_endian(value, &(rad_can_handle->TxData[0]));
    rad_can_handle->TxHeader.DLC = sizeof(uint32_t); //float
    rad_can_handle->TxHeader.ExtId = __encode_ext_can_id(rad_can_handle->id, message_id);

    HAL_CAN_AddTxMessage(&(rad_can_handle->hcan), &(rad_can_handle->TxHeader),
            rad_can_handle->TxData, &(rad_can_handle->TxMailbox));
}

void MX_CAN_Broadcast_Uint16_Data(RAD_CAN_TypeDef *rad_can_handle, uint16_t value, uint16_t message_id)
{
    encode_uint32_big_endian(value, &(rad_can_handle->TxData[0]));
    rad_can_handle->TxHeader.DLC = sizeof(uint16_t); //float
    rad_can_handle->TxHeader.ExtId = __encode_ext_can_id(rad_can_handle->id, message_id);

    HAL_CAN_AddTxMessage(&(rad_can_handle->hcan), &(rad_can_handle->TxHeader),
            rad_can_handle->TxData, &(rad_can_handle->TxMailbox));
}

void MX_CAN_Broadcast_Uint8_Data(RAD_CAN_TypeDef *rad_can_handle, uint8_t value, uint16_t message_id)
{
    rad_can_handle->TxData[0] = value;
    rad_can_handle->TxHeader.DLC = sizeof(uint8_t); //float
    rad_can_handle->TxHeader.ExtId = __encode_ext_can_id(rad_can_handle->id, message_id);

    HAL_CAN_AddTxMessage(&(rad_can_handle->hcan), &(rad_can_handle->TxHeader),
            rad_can_handle->TxData, &(rad_can_handle->TxMailbox));
}
// todo status return value?
// void MX_CAN_Broadcast_RAD_Status(RAD_CAN_TypeDef *rad_can_handle,
//         RAD_STATUS_TypeDef status)
// {
//     // status message 1
//     rad_can_handle->TxData[0] = ((status.fsr_2 & 0x03) << 1)
//             | ((status.fsr_1 & 0x01) << 2) | ((status.ls_2 & 0x01) << 1)
//             | (status.ls_1 & 0x01);
//     encode_float_big_endian(status.current_angle, &(rad_can_handle->TxData[1]));
//     rad_can_handle->TxHeader.DLC = 5;
//     // status message 1 is ID 9 according to VESC
//     // https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md
//     rad_can_handle->TxHeader.ExtId = __encode_ext_can_id(rad_can_handle->id, 9);
//     HAL_CAN_AddTxMessage(&(rad_can_handle->hcan), &(rad_can_handle->TxHeader),
//             rad_can_handle->TxData, &(rad_can_handle->TxMailbox));

//     // status message 2
//     encode_float_big_endian(status.kp, &(rad_can_handle->TxData[0]));
//     encode_float_big_endian(status.ki, &(rad_can_handle->TxData[4]));
//     rad_can_handle->TxHeader.DLC = 8;
//     rad_can_handle->TxHeader.ExtId = __encode_ext_can_id(rad_can_handle->id,
//             14);
//     HAL_CAN_AddTxMessage(&(rad_can_handle->hcan), &(rad_can_handle->TxHeader),
//             rad_can_handle->TxData, &(rad_can_handle->TxMailbox));

//     // status message 3
//     encode_float_big_endian(status.kd, &(rad_can_handle->TxData[0]));
// //    todo include rad motor speed
// //    encode_float_big_endian(status.speed, &(rad_can_handle->TxData[4]));
//     rad_can_handle->TxHeader.DLC = 4;
//     rad_can_handle->TxHeader.ExtId = __encode_ext_can_id(rad_can_handle->id,
//             15);
//     HAL_CAN_AddTxMessage(&(rad_can_handle->hcan), &(rad_can_handle->TxHeader),
//             rad_can_handle->TxData, &(rad_can_handle->TxMailbox));
// }

uint32_t __encode_ext_can_id(uint8_t device_id, uint8_t message_id)
{
    // return a value that combines both the device ID and the
    // message ID so that the message can be identified
    return (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) |
            (CAN_MESSAGE_RESPONSE_RAD << CAN_MESSAGE_RESPONSE_OFFSET) |
            (message_id << CAN_MESSAGE_COMMAND_OFFSET) | (device_id << CAN_MESSAGE_DEVICE_ID_OFFSET);
}

/* USER CODE END 1 */
