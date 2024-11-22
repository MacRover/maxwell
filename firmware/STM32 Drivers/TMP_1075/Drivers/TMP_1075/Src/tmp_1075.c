/*
 * tmp_1075.c
 *
 *  Created on: Nov 18, 2024
 *      Author: John
 */
#include "tmp_1075.h"

TMP_1075_StatusTypeDef TMP_1075_Init(TMP_1075_HandleTypeDef* tmp_1075)
{
	if (tmp_1075 == NULL)
	{
		return TMP_1075_ERROR;
	}
	if (tmp_1075->__hi2c == NULL)
	{
		return TMP_1075_ERROR;
	}
	return TMP_1075_OK;
}

TMP_1075_StatusTypeDef TMP_1075_DeInit(TMP_1075_HandleTypeDef* tmp_1075)
{
	tmp_1075->__hi2c = NULL;
	return TMP_1075_OK;
}

TMP_1075_StatusTypeDef TMP_1075_SetLowLimit(TMP_1075_HandleTypeDef* tmp_1075)
{
	uint8_t buf[3];
	HAL_StatusTypeDef ret;

	buf[0] = TMP_1075_LLIM;
	buf[1] = ((tmp_1075->low_limit & 0xff0) >> 4);
	buf[2] = ((tmp_1075->low_limit & 0xf) << 4);

	ret = HAL_I2C_Master_Transmit(tmp_1075->__hi2c, TMP_1075_ADDR, buf, 3, 1000);
	if (ret == HAL_ERROR)
	{
		return TMP_1075_ERROR;
	}
	if (ret == HAL_BUSY)
	{
		return TMP_1075_BUSY;
	}
	if (ret == HAL_TIMEOUT)
	{
		return TMP_1075_TIMEOUT;
	}
	return TMP_1075_OK;
}

TMP_1075_StatusTypeDef TMP_1075_SetHighLimit(TMP_1075_HandleTypeDef* tmp_1075)
{
	uint8_t buf[3];
	HAL_StatusTypeDef ret;

	buf[0] = TMP_1075_HLIM;
	buf[1] = ((tmp_1075->high_limit & 0xff0) >> 4);
	buf[2] = ((tmp_1075->high_limit & 0xf) << 4);

	ret = HAL_I2C_Master_Transmit(tmp_1075->__hi2c, TMP_1075_ADDR, buf, 3, 1000);
	if (ret == HAL_ERROR)
	{
		return TMP_1075_ERROR;
	}
	if (ret == HAL_BUSY)
	{
		return TMP_1075_BUSY;
	}
	if (ret == HAL_TIMEOUT)
	{
		return TMP_1075_TIMEOUT;
	}
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
	if (ret == HAL_BUSY)
	{
		return TMP_1075_BUSY;
	}
	if (ret == HAL_TIMEOUT)
	{
		return TMP_1075_TIMEOUT;
	}

	ret = HAL_I2C_Master_Receive(tmp_1075->__hi2c, TMP_1075_ADDR, buf, 2, 1000);
	if (ret == HAL_ERROR)
	{
		return TMP_1075_ERROR;
	}
	if (ret == HAL_BUSY)
	{
		return TMP_1075_BUSY;
	}
	if (ret == HAL_TIMEOUT)
	{
		return TMP_1075_TIMEOUT;
	}

	tmp_1075->temp = (int16_t)( (buf[1] >> 4) | ((uint16_t)buf[0] << 4) );

	return TMP_1075_OK;
}

TMP_1075_StatusTypeDef TMP_1075_SetConfRegisters(TMP_1075_HandleTypeDef* tmp_1075)
{
	uint8_t buf[3];
	HAL_StatusTypeDef ret;

	uint8_t conf_reg_high = 0;
	uint8_t conf_reg_low  = (1 << 7); // Low byte reserved

	conf_reg_high |= ((tmp_1075->conf.os & 0x01) << 7);
	conf_reg_high |= (0x03 << 5); // Conversation rate set to 250 ms
	conf_reg_high |= ((tmp_1075->conf.faults & 0x03) << 3);
	conf_reg_high |= ((tmp_1075->conf.polarity & 0x01) << 2);
	conf_reg_high |= ((tmp_1075->conf.tm & 0x01) << 1);
	conf_reg_high |= ((tmp_1075->conf.sd & 0x01) << 0);

	buf[0] = TMP_1075_CFGR;
	buf[1] = conf_reg_high;
	buf[2] = conf_reg_low;

	ret = HAL_I2C_Master_Receive(tmp_1075->__hi2c, TMP_1075_ADDR, buf, 3, 1000);
	if (ret == HAL_ERROR)
	{
		return TMP_1075_ERROR;
	}
	if (ret == HAL_BUSY)
	{
		return TMP_1075_BUSY;
	}
	if (ret == HAL_TIMEOUT)
	{
		return TMP_1075_TIMEOUT;
	}
	return TMP_1075_OK;
}

