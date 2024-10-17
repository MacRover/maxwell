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

typedef enum {
	RAD_TYPE_DRIVETRAIN,
	RAD_TYPE_ARM_BASE,
	RAD_TYPE_ARM_ELBOW,
	RAD_TYPE_ARM_WRIST,
	RAD_TYPE_ARM_GRIPPER,
} RAD_TYPE;


typedef struct __attribute__((packed)){
	uint8_t RAD_ID;
	RAD_TYPE RAD_TYPE;
	uint8_t HOME_POSITION;
	uint16_t STEPPER_SPEED;
	uint16_t ODOM_INTERVAL;
 	uint16_t HEALTH_INTERVAL;
	float P;
	float I;
	float D;
	uint8_t DRVCONF_TST;// : 1;
	uint8_t DRVCONF_SLP;// : 5;
	uint8_t DRVCONF_DIS_S2G;// : 1;
	uint8_t DRVCONF_TS2G;// : 2;
	uint8_t DRVCONF_SDOFF;// : 1;
	uint8_t DRVCONF_VSENSE;// : 1;
	uint8_t DRVCONF_RDSEL;// : 2;
	uint8_t DRVCONF_OTSENS;// : 1;
	uint8_t DRVCONF_SHRTSENS;// : 1;
	uint8_t DRVCONF_EN_PFD;// : 1;
	uint8_t DRVCONF_EN_S2VS;// : 1;
	uint8_t SGCSCONF_SFILT;// : 1;
	uint8_t SGCSCONF_SGT;// : 7;
	uint8_t SGCSCONF_CS;// : 8;
	uint8_t SMARTEN_SEIMIN;// : 1;
	uint16_t SMARTEN_SEDN;// : 13;
	uint8_t SMARTEN_SEMAX;// : 4;
	uint8_t SMARTEN_SEUP;// : 2;
	uint8_t SMARTEN_SEMIN;// : 4;
	uint8_t CHOPCONF_TBL;// : 2;
	uint8_t CHOPCONF_CHM;// : 1;
	uint8_t CHOPCONF_RNDTF;// : 1;
	uint8_t CHOPCONF_HDEC;// : 2;
	uint8_t CHOPCONF_HEND;// : 4;
	uint8_t CHOPCONF_HSTRT;// : 3;
	uint8_t CHOPCONF_TOFF;// : 4;
	uint8_t DRVCTRL_INTPOL;// : 1;
	uint8_t DRVCTRL_DEDGE;// : 1;
	uint8_t DRVCTRL_MRES;// : 4;
} RAD_PARAMS;


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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LS_1_Pin GPIO_PIN_0
#define LS_1_GPIO_Port GPIOA
#define LS_2_Pin GPIO_PIN_1
#define LS_2_GPIO_Port GPIOA
#define FSR_1_Pin GPIO_PIN_2
#define FSR_1_GPIO_Port GPIOA
#define FSR_2_Pin GPIO_PIN_3
#define FSR_2_GPIO_Port GPIOA
#define DRIVER_CS_Pin GPIO_PIN_4
#define DRIVER_CS_GPIO_Port GPIOA
#define DRIVER_SCK_Pin GPIO_PIN_5
#define DRIVER_SCK_GPIO_Port GPIOA
#define DRIVER_MISO_Pin GPIO_PIN_6
#define DRIVER_MISO_GPIO_Port GPIOA
#define DRIVER_MOSI_Pin GPIO_PIN_7
#define DRIVER_MOSI_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_2
#define LED_RED_GPIO_Port GPIOB
#define ENCODER_CS_Pin GPIO_PIN_12
#define ENCODER_CS_GPIO_Port GPIOB
#define ENCODER_SCK_Pin GPIO_PIN_13
#define ENCODER_SCK_GPIO_Port GPIOB
#define ENCODER_MISO_Pin GPIO_PIN_14
#define ENCODER_MISO_GPIO_Port GPIOB
#define ENCODER_MOSI_Pin GPIO_PIN_15
#define ENCODER_MOSI_GPIO_Port GPIOB
#define DRIVER_SG_TEST_Pin GPIO_PIN_9
#define DRIVER_SG_TEST_GPIO_Port GPIOA
#define DRIVER_ENN_Pin GPIO_PIN_10
#define DRIVER_ENN_GPIO_Port GPIOA
#define DRIVER_ST_ALONE_Pin GPIO_PIN_15
#define DRIVER_ST_ALONE_GPIO_Port GPIOA
#define DRIVER_STEP_PWM_Pin GPIO_PIN_3
#define DRIVER_STEP_PWM_GPIO_Port GPIOB
#define DRIVER_DIR_Pin GPIO_PIN_4
#define DRIVER_DIR_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define RAD_TYPE_DRIVETRAIN_MAX_ROTATIONS 20
#define RAD_TYPE_DRIVETRAIN_GEARING 30

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
