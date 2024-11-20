/*
 * tmp_1075.c
 *
 *  Created on: Nov 18, 2024
 *      Author: johnn
 */
#include "tmp_1075.h"

TMP_1075_StatusTypeDef TMP_1075_Init(TMP_1075_HandleTypeDef* tmp_1075)
{
	return TMP_1075_OK;
}

TMP_1075_StatusTypeDef TMP_1075_DeInit(TMP_1075_HandleTypeDef* tmp_1075)
{
	return TMP_1075_OK;
}

TMP_1075_StatusTypeDef TMP_1075_ReadTemp(TMP_1075_HandleTypeDef* tmp_1075)
{
	uint8_t buf[2];
	HAL_StatusTypeDef ret;

	buf[0] = TMP_1075_TEMP;
	ret = HAL_I2C_Master_Transmit(tmp_1075->__hi2c, TMP_1075_ADDR, buf, 1, 1000);
	if (ret == HAL_ERROR)
	{
		return TMP_1075_ERROR;
	}

	ret = HAL_I2C_Master_Receive(tmp_1075->__hi2c, TMP_1075_ADDR, buf, 2, 1000);
	if (ret == HAL_ERROR)
	{
		return TMP_1075_ERROR;
	}

	tmp_1075->temp = (int16_t)( (buf[1] >> 4) | ((uint16_t)(buf[0] << 4)) );

	return TMP_1075_OK;
}

