/*
 * tmp_1075.h
 *
 *  Created on: Nov 18, 2024
 *      Author: johnn
 */

#ifndef TMP_1075_INC_TMP_1075_H_
#define TMP_1075_INC_TMP_1075_H_

#include "stm32f1xx.h"

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
	TMP_1075_DIEID = 0x0F

} TMP_1075_I2C_Addresses;

typedef struct
{
	int16_t temp;
	uint8_t cfg_reg;
	int16_t low_limit_reg;
	int16_t high_limit_reg;

	I2C_HandleTypeDef* __hi2c;

} TMP_1075_HandleTypeDef;


TMP_1075_StatusTypeDef TMP_1075_Init(TMP_1075_HandleTypeDef* tmp_1075);

TMP_1075_StatusTypeDef TMP_1075_DeInit(TMP_1075_HandleTypeDef* tmp_1075);


#endif /* TMP_1075_INC_TMP_1075_H_ */
