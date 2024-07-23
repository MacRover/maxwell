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
    float current_angle;
//    float current_speed;

    uint8_t limit_switch_state;
    uint8_t upper_bound_state;

    float kp;
    float ki;
    float kd;
} RAD_status_TypeDef;

// todo enum of message IDs
//typedef enum {
//
//} RAD_CAN_MessageId;

extern RAD_CAN_TypeDef rad_can;

/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */
void MX_CAN_Broadcast_RAD_Status(RAD_CAN_TypeDef *rad_can_handle,
        RAD_status_TypeDef status);

uint32_t __encode_ext_can_id(uint8_t device_id, uint8_t message_id);

void __encode_float_big_endian(float value, uint8_t *data);

// todo decode can id to message id enum
//__decode_ext_can_id(uint32_t can_message_id);



/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

