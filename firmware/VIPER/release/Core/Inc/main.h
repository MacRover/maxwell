/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef enum
{
	VIPER_STATE_INIT,
	VIPER_STATE_ACTIVE
} VIPER_STATE_TypeDef;

typedef enum
{
	VIPER_CARD_OK,
	VIPER_CARD_FAULT,
	VIPER_CARD_DISCONNECTED
} VIPER_CARD_STATUS_TypeDef;

typedef enum
{
	VIPER_CARD0,
	VIPER_CARD1,
	VIPER_CARD2,
	VIPER_CARD3
} VIPER_CARD_ID_TypeDef;

typedef enum
{
	VIPER_LP,
	VIPER_HP
} VIPER_CARD_TYPE_TypeDef;

typedef struct
{
    //UPDATE THIS TO INCLUDE ERRORS
    //ENSURE EACH LIBRARY IS SENDING APPROPRIATE ERRORS

	// NEED TO FIGURE OUT HOW TO SEND THE DRIVER STATUS MESSAGES INSTEADD ; SEE RAD
//    uint8_t EEPROM_STATUS;
//    uint8_t MUX_STATUS;
    VIPER_CARD_STATUS_TypeDef CARD0_STATUS;
    VIPER_CARD_STATUS_TypeDef CARD1_STATUS;
    VIPER_CARD_STATUS_TypeDef CARD2_STATUS;
    VIPER_CARD_STATUS_TypeDef CARD3_STATUS;

} VIPER_STATUS_TypeDef;

typedef struct
{
	VIPER_CARD_ID_TypeDef VIPER_CARD_ID;
	VIPER_CARD_TYPE_TypeDef VIPER_CARD_TYPE;
	uint8_t VIPER_CARD_ENABLE;
	double VIPER_CARD_MAX_VOLTAGE;
	double VIPER_CARD_MAX_CURRENT;
	double VIPER_CARD_RESISTANCE;
} VIPER_CARD_TypeDef;

typedef struct __attribute__((packed))
{
 	uint16_t HEALTH_INTERVAL;
 	uint16_t CARD_INTERVAL;
 	VIPER_CARD_TypeDef VIPER_CARD0;
 	VIPER_CARD_TypeDef VIPER_CARD1;
 	VIPER_CARD_TypeDef VIPER_CARD2;
 	VIPER_CARD_TypeDef VIPER_CARD3;
} VIPER_PARAMS_TypeDef;

extern VIPER_PARAMS_TypeDef viper_params;
extern VIPER_STATE_TypeDef viper_state;
extern VIPER_CARD_ID_TypeDef viper_current_card;

typedef enum
{
	VIPER_OK,
	VIPER_ERROR,
	VIPER_TIMEOUT
} VIPER_ERROR_TypeDef;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

VIPER_ERROR_TypeDef VIPER_Card_Toggle(VIPER_PARAMS_TypeDef* viper_params, VIPER_CARD_ID_TypeDef cardx, uint8_t enable);
VIPER_ERROR_TypeDef VIPER_Card_Check(VIPER_PARAMS_TypeDef* viper_params);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IN_FAULT_CARD_0_Pin GPIO_PIN_0
#define IN_FAULT_CARD_0_GPIO_Port GPIOA
#define IN_FAULT_CARD_1_Pin GPIO_PIN_1
#define IN_FAULT_CARD_1_GPIO_Port GPIOA
#define IN_FAULT_CARD_2_Pin GPIO_PIN_2
#define IN_FAULT_CARD_2_GPIO_Port GPIOA
#define IN_FAULT_CARD_3_Pin GPIO_PIN_3
#define IN_FAULT_CARD_3_GPIO_Port GPIOA
#define PGOOD_CARD_0_Pin GPIO_PIN_4
#define PGOOD_CARD_0_GPIO_Port GPIOA
#define PGOOD_CARD_1_Pin GPIO_PIN_5
#define PGOOD_CARD_1_GPIO_Port GPIOA
#define PGOOD_CARD_2_Pin GPIO_PIN_6
#define PGOOD_CARD_2_GPIO_Port GPIOA
#define PGOOD_CARD_3_Pin GPIO_PIN_7
#define PGOOD_CARD_3_GPIO_Port GPIOA
#define OUT_FAULT_5_Pin GPIO_PIN_0
#define OUT_FAULT_5_GPIO_Port GPIOB
#define OUT_FAULT_4_Pin GPIO_PIN_1
#define OUT_FAULT_4_GPIO_Port GPIOB
#define OUT_FAULT_3_Pin GPIO_PIN_2
#define OUT_FAULT_3_GPIO_Port GPIOB
#define OUT_FAULT_2_Pin GPIO_PIN_12
#define OUT_FAULT_2_GPIO_Port GPIOB
#define OUT_FAULT_1_Pin GPIO_PIN_13
#define OUT_FAULT_1_GPIO_Port GPIOB
#define OUT_FAULT_0_Pin GPIO_PIN_14
#define OUT_FAULT_0_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_15
#define LED_GPIO_Port GPIOB
#define SM_ALERT_Pin GPIO_PIN_3
#define SM_ALERT_GPIO_Port GPIOB
#define EN_CARD_0_Pin GPIO_PIN_4
#define EN_CARD_0_GPIO_Port GPIOB
#define EN_CARD_1_Pin GPIO_PIN_5
#define EN_CARD_1_GPIO_Port GPIOB
#define EN_CARD_2_Pin GPIO_PIN_8
#define EN_CARD_2_GPIO_Port GPIOB
#define EN_CARD_3_Pin GPIO_PIN_9
#define EN_CARD_3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
