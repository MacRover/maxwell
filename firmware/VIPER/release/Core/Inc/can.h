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

#define CAN_MESSAGE_DEVICE_ID_MASK 0x3F
#define CAN_MESSAGE_DEVICE_ID_OFFSET 2

#define CAN_MESSAGE_CARD_ID_MASK 0x3
#define CAN_MESSAGE_CARD_ID_OFFSET 0

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

	// todo (combine): replace each card enum variant command with a reserved 2 bits in the address to select each card instead
	// todo : add command ids for sending connected, enable, and ?status fields

	// TO VIPER (LOCAL)

	DISABLE_CARD = 0x00,
	DISABLE_ALL_CARDS = 0x01,
	ENABLE_CARD = 0x02,
	ENABLE_ALL_CARDS = 0x03,
	GET_CARD_DATA = 0x04,
	GET_ALL_CARD_DATA = 0x05,
	SET_MUX_VALUE = 0x06,
	SAVE_TO_EEPROM = 0x07,
	SET_FREEZE = 0x08,
	STOP_FREEZE = 0x09,

	// GETTERS / SETTERS

	GET_HEALTH_INTERVAL = 0x0A,
	SET_HEALTH_INTERVAL = 0x0B,
	GET_CARD_INTERVAL = 0x0D,
	SET_CARD_INTERVAL = 0x0E,

	// FROM VIPER
    SEND_CARD_INPUT_VOLTAGE = 0xF1,
	SEND_CARD_INPUT_FAULT = 0xF2,
	SEND_CARD_OUTPUT_A_FAULT = 0xF3,
	SEND_CARD_OUTPUT_B_FAULT = 0xF4,
	SEND_CARD_TEMPERATURE = 0xF5,
	SEND_CARD_INPUT_CURRENT = 0xF6,
	SEND_CARD_OUTPUT_DIAGNOSTIC_A = 0xF7,
    SEND_CARD_OUTPUT_POWER_A = 0xF8,
	SEND_CARD_OUTPUT_CURRENT_A = 0xF9,
	SEND_CARD_OUTPUT_VOLTAGE_A = 0xFA,
	SEND_CARD_OUTPUT_DIAGNOSTIC_B = 0xFB,
    SEND_CARD_OUTPUT_POWER_B = 0xFC,
	SEND_CARD_OUTPUT_CURRENT_B = 0xFD,
	SEND_CARD_OUTPUT_VOLTAGE_B = 0xFE,
	SEND_HEALTH_STATUS = 0xFF

} VIPER_CAN_CommandId;

typedef struct
{
    VIPER_CAN_CommandId command_id;
    uint32_t ext_id; //Used for TX 
    uint8_t *data;
    uint8_t dlc;
    VIPER_CARD_ID_TypeDef card_id;
} VIPER_CAN_Message_TypeDef;

extern VIPER_CAN_TypeDef viper_can;

/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */

void MX_CAN_UpdateIdAndFilters(VIPER_CAN_TypeDef *viper_can_handle);

void MX_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t *aData, uint32_t *pTxMailbox); //Keep this private

void MX_CAN_Broadcast_Card_Data(VIPER_CAN_TypeDef *viper_can_handle, VIPER_STATE_TypeDef* viper_state, VIPER_CARD_ID_TypeDef cardx);
void MX_CAN_Broadcast_Health_Message(VIPER_CAN_TypeDef *viper_can_handle, VIPER_STATE_TypeDef *viper_state);

void MX_CAN_Broadcast_Double_Data(VIPER_CAN_TypeDef *viper_can_handle, double value, uint16_t message_id, VIPER_CARD_ID_TypeDef card_id);
void MX_CAN_Broadcast_Uint32_Data(VIPER_CAN_TypeDef *viper_can_handle, uint32_t value, uint16_t message_id, VIPER_CARD_ID_TypeDef card_id);
void MX_CAN_Broadcast_Uint16_Data(VIPER_CAN_TypeDef *viper_can_handle, uint16_t value, uint16_t message_id, VIPER_CARD_ID_TypeDef card_id);
void MX_CAN_Broadcast_Uint8_Data(VIPER_CAN_TypeDef *viper_can_handle, uint8_t value, uint16_t message_id, VIPER_CARD_ID_TypeDef card_id);

uint32_t __encode_ext_can_id(uint8_t device_id, uint8_t message_id, VIPER_CARD_ID_TypeDef card_id);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

