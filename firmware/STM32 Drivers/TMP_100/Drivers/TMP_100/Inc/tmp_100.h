/*
 * tmp_100.h
 *
 *  Created on: Nov 18, 2024
 *      Author: John
 */

#ifndef TMP_100_INC_TMP_100_H_
#define TMP_100_INC_TMP_100_H_

#include "stm32f1xx.h"

#define TMP_100_ADDR (uint16_t)(0x48 << 1)

#define MIN_TEMP_C -50.0
#define MAX_TEMP_C 125.0

typedef enum
{
    TMP_100_OK = 0x00U,
    TMP_100_ERROR = 0x01U,
    TMP_100_BUSY = 0x02U,
    TMP_100_TIMEOUT = 0x03U
} TMP_100_StatusTypeDef;

typedef enum
{
	TMP_100_TEMP  = 0x00,
	TMP_100_CFGR  = 0x01,
	TMP_100_LLIM  = 0x02,
	TMP_100_HLIM  = 0x03,
//	TMP_100_DIEID = 0x0F

} TMP_100_I2C_Addresses;

typedef struct
{
	uint8_t os;       // One shot conversion mode (1 for single, 0 for continuous)
	uint8_t res;	  // Resolution of system (00b: 9 bits, 01b: 10 bits, 10b: 11 bits, 11b: 12 bits)
	uint8_t faults;   // Number of faults for alert to trigger (00b: 1, 01b: 2, 10b: 4, 11b: 6)
	uint8_t sd;       // Shutdown mode (0 for continuous, 1 for shutdown/single)

} TMP_100_ConfTypeDef;

typedef struct
{
	float temp;              // Current temperature reading (in C)
	float low_limit;         // low limit temperature
	float high_limit;        // high limit temperature

	TMP_100_ConfTypeDef conf; // configuration

	I2C_HandleTypeDef* __hi2c;

	uint8_t a0_pin;
	uint8_t a1_pin;

} TMP_100_HandleTypeDef;


TMP_100_StatusTypeDef TMP_100_Init(TMP_100_HandleTypeDef* tmp_100);

TMP_100_StatusTypeDef TMP_100_DeInit(TMP_100_HandleTypeDef* tmp_100);

TMP_100_StatusTypeDef TMP_100_ReadTemp(TMP_100_HandleTypeDef* tmp_100);

TMP_100_StatusTypeDef TMP_100_SetLowLimit(TMP_100_HandleTypeDef* tmp_100);

TMP_100_StatusTypeDef TMP_100_SetHighLimit(TMP_100_HandleTypeDef* tmp_100);

TMP_100_StatusTypeDef TMP_100_SetConfRegisters(TMP_100_HandleTypeDef* tmp_100);

double __scale_factor(TMP_100_HandleTypeDef* tmp_100);


#endif /* TMP_100_INC_TMP_100_H_ */
