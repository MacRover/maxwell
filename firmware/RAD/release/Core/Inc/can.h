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

    uint8_t ls_1;
    uint8_t ls_2;
    uint8_t fsr_1;
    uint8_t fsr_2;

    float kp;
    float ki;
    float kd;
} RAD_status_TypeDef;

// enum of message IDs

typedef enum
{
    ESTOP_MESSAGE = 0x0,
    DISABLE_MESSAGE = 0x01,
    ENABLE_MESSAGE = 0x02
} GLOBAL_CAN_CommandId;

typedef enum
{
   
    // match VESC tool
    SET_TARGET_ANGLE = 0x4,
//    SET_STEPPER_SPEED = 0x3,
//    PACKET_PING = 0x11,
    UPDATE_PID_POS_OFFSET = 0x37, // sets current position as the 0 offset point
    // Custom
//    SET_RAD_FLAGS = 0x40,
//    SET_WATCHDOG_INTERVAL = 0x41,
//    SAVE_TO_EEPROM = 0x42,
    CALIBRATE_PID_POS_OFFSET = 0x43, // moves motor until it hits limit switch and then sets as zero point
    SET_CAN_ID = 0x44,
//    SET_OUTPUT_RATIO = 0x45,
    SET_P_VALUE = 0x46,
    SET_I_VALUE = 0x47,
    SET_D_VALUE = 0x48,
    RESET_BOARD = 0X49,
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
void MX_CAN_Broadcast_RAD_Status(RAD_CAN_TypeDef *rad_can_handle,
        RAD_status_TypeDef status);

uint32_t __encode_ext_can_id(uint8_t device_id, uint8_t message_id);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

