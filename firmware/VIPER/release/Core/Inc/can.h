/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */

#define CAN_MESSAGE_IDENTIFIER_VIPER 0x05 // To be changed upon VIPER introduction
#define CAN_MESSAGE_IDENTIFIER_GLOBAL 0x00
#define CAN_MESSAGE_IDENTIFIER_MASK 0b1111
#define CAN_MESSAGE_IDENTIFIER_OFFSET 25

#define CAN_MESSAGE_RESPONSE_VIPER 0x01
#define CAN_MESSAGE_RESPONSE_OFFSET 18

#define CAN_MESSAGE_COMMAND_MASK 0xFF
#define CAN_MESSAGE_COMMAND_OFFSET 8

#define CAN_MESSAGE_DEVICE_ID_MASK 0xFF
#define CAN_MESSAGE_DEVICE_ID_OFFSET 0

typedef struct
{
    CAN_HandleTypeDef hcan;
    uint8_t id;

    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    CAN_FilterTypeDef canfilterconfig;
    CAN_FilterTypeDef canfilter_global1;
    CAN_FilterTypeDef canfilter_global2;
} VIPER_CAN_TypeDef;

// enum of message IDs

typedef enum
{
    ESTOP_MESSAGE = 0x31,
    DISABLE_MESSAGE = 0x00,
    ENABLE_MESSAGE = 0x02,
    HEALTH_STATUS_PING = 0x03,
} GLOBAL_CAN_CommandId;

typedef enum
{
	// STATUS / EEPROM MESSAGES

	GET_CARD_1_STATUS = 0x01,
	GET_CARD_2_STATUS = 0x02,
	GET_CARD_3_STATUS = 0x03,
	GET_CARD_4_STATUS = 0x04,
	SYNC_EEPROM = 0x05,
	CLEAR_EEPROM = 0x06,
	GET_EEPROM_STATUS = 0x07,
    GET_HEALTH_STATUS = 0x08,
	GET_MUX_STATUS = 0x09,

	// GETTERS / SETTERS

	GET_HEALTH_INTERVAL = 0x11,
	SET_HEALTH_INTERVAL = 0x21,
	GET_CARD_INTERVAL = 0x12,
	SET_CARD_INTERVAL = 0x22,
	GET_VIPER_ID = 0x13,
	SET_VIPER_ID = 0x23,

} VIPER_CAN_CommandId;

typedef struct
{
    VIPER_CAN_CommandId command_id;
    uint8_t *data;
    uint8_t dlc;
} VIPER_CAN_Message_TypeDef;

extern VIPER_CAN_TypeDef viper_can;

/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */

void MX_CAN_UpdateIdAndFilters(VIPER_CAN_TypeDef *viper_can_handle);

void MX_CAN_Broadcast_Card_Data(VIPER_CAN_TypeDef *viper_can_handle, VIPER_PARAMS_TypeDef* viper_params, VIPER_CARD_ID_TypeDef cardx);
void MX_CAN_Broadcast_Health_Message(VIPER_CAN_TypeDef *viper_can_handle, VIPER_STATUS_TypeDef status);

void MX_CAN_Broadcast_Double_Data(VIPER_CAN_TypeDef *viper_can_handle, double value, uint16_t message_id);
void MX_CAN_Broadcast_Uint32_Data(VIPER_CAN_TypeDef *viper_can_handle, uint32_t value, uint16_t message_id);
void MX_CAN_Broadcast_Uint16_Data(VIPER_CAN_TypeDef *viper_can_handle, uint16_t value, uint16_t message_id);
void MX_CAN_Broadcast_Uint8_Data(VIPER_CAN_TypeDef *viper_can_handle, uint8_t value, uint16_t message_id);

uint32_t __encode_ext_can_id(uint8_t device_id, uint8_t message_id);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */
