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

	// todo (combine): replace each card enum variant command with a reserved 2 bits in the address to select each card instead
	// todo : add command ids for sending connected, enable, and ?status fields

	// TO VIPER (LOCAL)

	DISABLE_CARD_0 = 0x00,
	DISABLE_CARD_1 = 0x01,
	DISABLE_CARD_2 = 0x02,
	DISABLE_CARD_3 = 0x03,
	ENABLE_CARD_0 = 0x04,
	ENABLE_CARD_1 = 0x05,
	ENABLE_CARD_2 = 0x06,
	ENABLE_CARD_3 = 0x07,
	SET_MUX_VALUE = 0x08,
	GET_CARD_0_DATA = 0x09,
	GET_CARD_1_DATA = 0x0A,
	GET_CARD_2_DATA = 0x0B,
	GET_CARD_3_DATA = 0x0C,
	SAVE_TO_EEPROM = 0x0D,
	CLEAR_EEPROM = 0x0E,

	// GETTERS / SETTERS

	GET_HEALTH_INTERVAL = 0x0F,
	SET_HEALTH_INTERVAL = 0x10,
	SEND_HEALTH_INTERVAL = 0x11,
	GET_CARD_INTERVAL = 0x12,
	SET_CARD_INTERVAL = 0x13,
	SEND_CARD_INTERVAL = 0x14,

	// FROM VIPER

	SEND_CARD_0_INPUT_FAULT = 0xDB,
	SEND_CARD_0_OUTPUT_A_FAULT = 0xDC,
	SEND_CARD_0_OUTPUT_B_FAULT = 0xDD,
	SEND_CARD_1_INPUT_FAULT = 0xDE,
	SEND_CARD_1_OUTPUT_FAULT = 0xDF,
	SEND_CARD_2_INPUT_FAULT = 0xE0,
	SEND_CARD_2_OUTPUT_FAULT = 0xE1,
	SEND_CARD_3_INPUT_FAULT = 0xE2,
	SEND_CARD_3_OUTPUT_A_FAULT = 0xE3,
	SEND_CARD_3_OUTPUT_B_FAULT = 0xE4,
	SEND_CARD_0_TEMPERATURE = 0xE5,
	SEND_CARD_0_INPUT_CURRENT = 0xE6,
	SEND_CARD_0_OUTPUT_DIAGNOSTIC_A = 0xE7,
	SEND_CARD_0_OUTPUT_CURRENT_A = 0xE8,
	SEND_CARD_0_OUTPUT_VOLTAGE_A = 0xE9,
	SEND_CARD_0_OUTPUT_DIAGNOSTIC_B = 0xEA,
	SEND_CARD_0_OUTPUT_CURRENT_B = 0xEB,
	SEND_CARD_0_OUTPUT_VOLTAGE_B = 0xEC,
	SEND_CARD_1_TEMPERATURE = 0xED,
	SEND_CARD_1_INPUT_CURRENT = 0xEE,
	SEND_CARD_1_OUTPUT_DIAGNOSTIC = 0xEF,
	SEND_CARD_1_OUTPUT_CURRENT = 0xF0,
	SEND_CARD_1_OUTPUT_VOLTAGE = 0xF1,
	SEND_CARD_2_TEMPERATURE = 0xF2,
	SEND_CARD_2_INPUT_CURRENT = 0xF3,
	SEND_CARD_2_OUTPUT_DIAGNOSTIC = 0xF4,
	SEND_CARD_2_OUTPUT_CURRENT = 0xF5,
	SEND_CARD_2_OUTPUT_VOLTAGE = 0xF6,
	SEND_CARD_3_TEMPERATURE = 0xF7,
	SEND_CARD_3_INPUT_CURRENT = 0xF8,
	SEND_CARD_3_OUTPUT_DIAGNOSTIC_A = 0xF9,
	SEND_CARD_3_OUTPUT_CURRENT_A = 0xFA,
	SEND_CARD_3_OUTPUT_VOLTAGE_A = 0xFB,
	SEND_CARD_3_OUTPUT_DIAGNOSTIC_B = 0xFC,
	SEND_CARD_3_OUTPUT_CURRENT_B = 0xFD,
	SEND_CARD_3_OUTPUT_VOLTAGE_B = 0xFE,
	SEND_HEALTH_STATUS = 0xFF,

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

void MX_CAN_Broadcast_Card_Data(VIPER_CAN_TypeDef *viper_can_handle, VIPER_STATE_TypeDef* viper_state, VIPER_CARD_ID_TypeDef cardx);
void MX_CAN_Broadcast_Health_Message(VIPER_CAN_TypeDef *viper_can_handle, VIPER_STATE_TypeDef *viper_state);

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

