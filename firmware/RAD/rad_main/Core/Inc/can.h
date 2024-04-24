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

/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */

enum CAN_STATUS {
	CAN_OK = 0,
	CAN_ERROR_NOT_INITIALIZED,
	CAN_ERROR_INVALID_ARGUMENT,
	CAN_ERROR_DATA,
	CAN_ERROR_BUFFER_FULL
};

struct CAN_MESSAGE {
	uint32_t header;
	uint8_t[8] data;
};

CAN_STATUS CAN_Initialize(void);
CAN_STATUS CAN_ConfigFilterId(uint8_t id);
CAN_STATUS CAN_ResetFilter(void);
CAN_STATUS CAN_Send(CAN_MESSAGE *msg);
CAN_STATUS CAN_RegisterReceiveCallback(void (*fcn)(CAN_MESSAGE *msg));
CAN_STATUS CAN_Deinitialize(void);




/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

