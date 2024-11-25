/*
 * tmp_1075.h
 *
 *  Created on: Nov 18, 2024
 *      Author: John
 */

#ifndef TMP_1075_INC_TMP_1075_H_
#define TMP_1075_INC_TMP_1075_H_

#include "stm32f1xx.h"

#define TMP_1075_ADDR (uint16_t)(0x48 << 1)

#define MIN_TEMP_C -50.0
#define MAX_TEMP_C 125.0

typedef enum
{
    TMP_1075_OK = 0x00U,
    TMP_1075_ERROR = 0x01U,
    TMP_1075_BUSY = 0x02U,
    TMP_1075_TIMEOUT = 0x03U
} TMP_1075_StatusTypeDef;

typedef enum
{
	TMP_1075_TEMP  = 0x00,
	TMP_1075_CFGR  = 0x01,
	TMP_1075_LLIM  = 0x02,
	TMP_1075_HLIM  = 0x03,
//	TMP_1075_DIEID = 0x0F

} TMP_1075_I2C_Addresses;

typedef struct
{
	uint8_t os;       // One shot conversion mode (1 for single, 0 for continuous)
	uint8_t faults;   // Number of faults for alert to trigger (00b: 1, 01b: 2, 10b: 4, 11b: 6)
	uint8_t polarity; // Polarity of the output pin (0 for comparator mode, 1 for interrupt mode)
	uint8_t tm;       // Function of alert pin (0 for comparator mode, 1 for interrupt mode)
	uint8_t sd;       // Shutdown mode (0 for continuous, 1 for shutdown/single)

} TMP_1075_ConfTypeDef;

typedef struct
{
	float temp;              // Current temperature reading (in C)
	float low_limit;         // low limit temperature
	float high_limit;        // high limit temperature

	TMP_1075_ConfTypeDef conf; // configuration

	I2C_HandleTypeDef* __hi2c;

	uint8_t a0_pin;

} TMP_1075_HandleTypeDef;


TMP_1075_StatusTypeDef TMP_1075_Init(TMP_1075_HandleTypeDef* tmp_1075);

TMP_1075_StatusTypeDef TMP_1075_DeInit(TMP_1075_HandleTypeDef* tmp_1075);

TMP_1075_StatusTypeDef TMP_1075_ReadTemp(TMP_1075_HandleTypeDef* tmp_1075);

TMP_1075_StatusTypeDef TMP_1075_SetLowLimit(TMP_1075_HandleTypeDef* tmp_1075);

TMP_1075_StatusTypeDef TMP_1075_SetHighLimit(TMP_1075_HandleTypeDef* tmp_1075);

TMP_1075_StatusTypeDef TMP_1075_SetConfRegisters(TMP_1075_HandleTypeDef* tmp_1075);

#endif /* TMP_1075_INC_TMP_1075_H_ */
