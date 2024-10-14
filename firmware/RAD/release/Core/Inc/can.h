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

#define CAN_MESSAGE_IDENTIFIER_RAD 0x02
#define CAN_MESSAGE_IDENTIFIER_GLOBAL 0x00
#define CAN_MESSAGE_IDENTIFIER_MASK 0b1111
#define CAN_MESSAGE_IDENTIFIER_OFFSET 25

#define CAN_MESSAGE_RESPONSE_RAD 0x01
#define CAN_MESSAGE_RESPONSE_OFFSET 18

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
} RAD_CAN_TypeDef;

typedef struct
{
    //UPDATE THIS TO INCLUDE ERRORS
    //ENSURE EACH LIBRARY IS SENDING APPROPRIATE ERRORS 
    float current_angle;
//    float current_speed;

    uint8_t EEPROM_STATUS;
    uint8_t TMC_STATUS;
    uint8_t ENCODER_STATUS;
    uint8_t RAD_STATE;
    uint8_t ls_1;

} RAD_status_TypeDef;

// enum of message IDs

typedef enum
{
    ESTOP_MESSAGE = 0x0,
    DISABLE_MESSAGE = 0x01,
    ENABLE_MESSAGE = 0x02,
    HEALTH_STATUS_PING = 0x03,
    ASSIGN_DEVICE_ID = 0x04
} GLOBAL_CAN_CommandId;

typedef enum
{
   
    SET_TARGET_ANGLE = 0x01,
    GET_ENCODER_VALUE = 0x02,
    SET_STEPPER_SPEED = 0x03,
    GET_STEPPER_SPEED = 0x04,
    SET_P_VALUE = 0x05,
    GET_P_VALUE = 0x06,
    SET_I_VALUE = 0x07,
    GET_I_VALUE = 0x08,
    SET_D_VALUE = 0x09,
    GET_D_VALUE = 0x10,
    SET_DRVCTRL_REGISTER = 0x11,
    GET_DRVCTRL_REGISTER = 0x12,
    SET_CHOPCONF_REGISTER = 0x13,
    GET_CHOPCONF_REGISTER = 0x14,
    SET_SMARTEN_REGISTER = 0x15,
    GET_SMARTEN_REGISTER = 0x16,
    SET_SGSCONF_REGISTER = 0x17,
    GET_SGSCONF_REGISTER = 0x18,
    SET_DRVCONF_REGISTER = 0x19,
    GET_DRVCONF_REGISTER = 0x20,
    SET_RAD_TYPE = 0x21,
    GET_RAD_TYPE = 0x22,
    SET_HOME_POSITION = 0x23,
    GET_HOME_POSITION = 0x24,
    SET_ODOM_INTERVAL = 0x35,
    GET_ODOM_INTERVAL = 0x36,
    SAVE_TO_EEPROM = 0x37,
    SET_HEALTH_INTERVAL = 0x39,
    GET_HEALTH_INTERVAL = 0x40,
    START_CALIBRATION_ROUTINE = 0x41,
    CANCEL_CALIBRATION_ROUTINE = 0x42,
    
    SEND_ODOM_ANGLE = 0xFB,
    SEND_HEALTH_STATUS = 0xFC
} RAD_CAN_CommandId;

typedef struct
{
    RAD_CAN_CommandId command_id;
    uint8_t *data;
    uint8_t dlc;
} RAD_CAN_Message_TypeDef;

extern RAD_CAN_TypeDef rad_can;

/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */

void MX_CAN_Broadcast_Odometry_Message(RAD_CAN_TypeDef *rad_can_handle, RAD_status_TypeDef status);

void MX_CAN_Broadcast_Health_Message(RAD_CAN_TypeDef *rad_can_handle, RAD_status_TypeDef status);

void MX_CAN_Broadcast_Double_Data(RAD_CAN_TypeDef *rad_can_handle, double value, uint16_t message_id);
void MX_CAN_Broadcast_Uint32_Data(RAD_CAN_TypeDef *rad_can_handle, uint32_t value, uint16_t message_id);


void MX_CAN_Broadcast_RAD_Status(RAD_CAN_TypeDef *rad_can_handle,
        RAD_status_TypeDef status);

uint32_t __encode_ext_can_id(uint8_t device_id, uint8_t message_id);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

