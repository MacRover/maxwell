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

#define CAN_MESSAGE_COMMAND_MASK 0xFF
#define CAN_MESSAGE_COMMAND_OFFSET 8

#define CAN_MESSAGE_DEVICE_ID_MASK 0xFF
#define CAN_MESSAGE_DEVICE_ID_OFFSET 0

#define CAN_MESSAGE_TIMEOUT_MS 1000

typedef struct
{
    CAN_HandleTypeDef hcan;
    uint8_t id;

    uint32_t timer;
    uint8_t watchdog_kick;

    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    CAN_FilterTypeDef canfilterconfig;
    CAN_FilterTypeDef canfilter_global1;
    CAN_FilterTypeDef canfilter_global2;
} RAD_CAN_TypeDef;

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
   
    SET_TARGET_ANGLE = 0x01,
    GET_ENCODER_VALUE = 0x02,
    SET_STEPPER_SPEED = 0x03,
    GET_STEPPER_SPEED = 0x04,
    SET_P_VALUE = 0x05,
    GET_P_VALUE = 0x06,
    SET_I_VALUE = 0x07,
    GET_I_VALUE = 0x08,
    SET_D_VALUE = 0x09,
    GET_D_VALUE = 0x0A,
    SET_RAD_TYPE = 0x0B,
    GET_RAD_TYPE = 0x0C,
    SET_HOME_POSITION = 0x0D,
    GET_HOME_POSITION = 0x0E,
    SET_ODOM_INTERVAL = 0x0F,
    GET_ODOM_INTERVAL = 0x10,
    SAVE_TO_EEPROM = 0x11,
    RELOAD_FROM_EEPROM = 0x12,
    SET_HEALTH_INTERVAL = 0x13,
    GET_HEALTH_INTERVAL = 0x14,
    START_CALIBRATE = 0x15,
    CANCEL_CALIBRATION = 0x16,
    SET_DRVCONF_TST = 0x17,
    GET_DRVCONF_TST = 0x18,
    SET_DRVCONF_SLP = 0x19,
    GET_DRVCONF_SLP = 0x1A,
    SET_DRVCONF_DIS_S2G = 0x1B,
    GET_DRVCONF_DIS_S2G = 0x1C,
    SET_DRVCONF_TS2G = 0x1D,
    GET_DRVCONF_TS2G = 0x1E,
    SET_DRVCONF_SDOFF = 0x1F,
    GET_DRVCONF_SDOFF = 0x20,
    SET_DRVCONF_VSENSE = 0x21,
    GET_DRVCONF_VSENSE = 0x22,
    SET_DRVCONF_RDSEL = 0x23,
    GET_DRVCONF_RDSEL = 0x24,
    SET_DRVCONF_OTSENS = 0x25,
    GET_DRVCONF_OTSENS = 0x26,
    SET_DRVCONF_SHRTSENS = 0x27,
    GET_DRVCONF_SHRTSENS = 0x28,
    SET_DRVCONF_EN_PFD = 0x29,
    GET_DRVCONF_EN_PFD = 0x2A,
    SET_DRVCONF_EN_S2VS = 0x2B,
    GET_DRVCONF_EN_S2VS = 0x2C,
    SET_SGCSCONF_SFILT = 0x2D,
    GET_SGCSCONF_SFILT = 0x2E,
    SET_SGCSCONF_SGT = 0x2F,
    GET_SGCSCONF_SGT = 0x30,
    SET_SGCSCONF_CS = 0x31,
    GET_SGCSCONF_CS = 0x32,
    SET_SMARTEN_SEIMIN = 0x33,
    GET_SMARTEN_SEIMIN = 0x34,
    SET_SMARTEN_SEDN = 0x35,
    GET_SMARTEN_SEDN = 0x36,
    SET_SMARTEN_SEMAX = 0x37,
    GET_SMARTEN_SEMAX = 0x38,
    SET_SMARTEN_SEUP = 0x39,
    GET_SMARTEN_SEUP = 0x3A,
    SET_SMARTEN_SEMIN = 0x3B,
    GET_SMARTEN_SEMIN = 0x3C,
    SET_CHOPCONF_TBL = 0x3D,
    GET_CHOPCONF_TBL = 0x3E,
    SET_CHOPCONF_CHM = 0x3F,
    GET_CHOPCONF_CHM = 0x40,
    SET_CHOPCONF_RNDTF = 0x41,
    GET_CHOPCONF_RNDTF = 0x42,
    SET_CHOPCONF_HDEC = 0x43,
    GET_CHOPCONF_HDEC = 0x44,
    SET_CHOPCONF_HEND = 0x45,
    GET_CHOPCONF_HEND = 0x46,
    SET_CHOPCONF_HSTRT = 0x47,
    GET_CHOPCONF_HSTRT = 0x48,
    SET_CHOPCONF_TOFF = 0x49,
    GET_CHOPCONF_TOFF = 0x4A,
    SET_DRVCTRL_INTPOL = 0x4B,
    GET_DRVCTRL_INTPOL = 0x4C,
    SET_DRVCTRL_DEDGE = 0x4D,
    GET_DRVCTRL_DEDGE = 0x4E,
    SET_DRVCTRL_MRES = 0x4F,
    GET_DRVCTRL_MRES = 0x50,
    PULSE_STEPPER = 0x51,
	REBOOT = 0x53,
    ASSIGN_DEVICE_ID = 0x55,
    SET_PID_MIN_OUTPUT = 0x57,
    GET_PID_MIN_OUTPUT = 0x58,
    SET_PID_MAX_OUTPUT = 0x59,
    GET_PID_MAX_OUTPUT = 0x5A,
    SET_HOME_OFFSET = 0x5B,
    GET_HOME_OFFSET = 0x5C,
	SET_RAD_FLAGS = 0x5D,
	GET_RAD_FLAGS = 0x5E,

    
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

void MX_CAN_UpdateIdAndFilters(RAD_CAN_TypeDef *rad_can_handle);

void MX_CAN_Broadcast_Odometry_Message(RAD_CAN_TypeDef *rad_can_handle, RAD_STATUS_TypeDef status);

void MX_CAN_Broadcast_Health_Message(RAD_CAN_TypeDef *rad_can_handle, RAD_STATUS_TypeDef status);

void MX_CAN_Broadcast_Double_Data(RAD_CAN_TypeDef *rad_can_handle, double value, uint16_t message_id);
void MX_CAN_Broadcast_Uint32_Data(RAD_CAN_TypeDef *rad_can_handle, uint32_t value, uint16_t message_id);
void MX_CAN_Broadcast_Uint16_Data(RAD_CAN_TypeDef *rad_can_handle, uint16_t value, uint16_t message_id);
void MX_CAN_Broadcast_Uint8_Data(RAD_CAN_TypeDef *rad_can_handle, uint8_t value, uint16_t message_id);

void MX_CAN_Broadcast_RAD_Status(RAD_CAN_TypeDef *rad_can_handle,
        RAD_STATUS_TypeDef status);

uint32_t __encode_ext_can_id(uint8_t device_id, uint8_t message_id);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

