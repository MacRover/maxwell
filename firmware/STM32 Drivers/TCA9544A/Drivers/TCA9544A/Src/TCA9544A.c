/*
 * TCA9548A.c
 *
 *  Created on: Nov 19, 2024
 *      Author: Adam and Vaibhav
 */

#include "TCA9544A.h"

TCA9544A_StatusTypeDef TCA9544A_Init(TCA9544A_HandleTypeDef *device) {
	if (device == NULL) {
		return TCA9544A_ERROR;
	}
	else if (device->__hi2c == NULL){
		return TCA9544A_ERROR;
	}
	return TCA9544A_OK;
}

TCA9544A_StatusTypeDef TCA9544A_SelectChannel(TCA9544A_HandleTypeDef *device, TCA9544A_ChannelSelect channel) {
	device->current_channel = channel;
	HAL_StatusTypeDef i2c_status;
	TCA9544A_StatusTypeDef status;

	if (device->current_channel == CHANNEL_NONE) {
		uint8_t register_write_buffer = 0b00000000;
		i2c_status = HAL_I2C_Master_Transmit(device->__hi2c, (TCA9544A_DevAddr | device->A2 << 3 | device->A1 << 2 | device->A0 << 1 | 0), &register_write_buffer, 1, 1000);
	} else {
		uint8_t register_write_buffer = 0b00000100 | device->current_channel;
		i2c_status = HAL_I2C_Master_Transmit(device->__hi2c, (TCA9544A_DevAddr | device->A2 << 3 | device->A1 << 2 | device->A0 << 1 | 0), &register_write_buffer, 1, 1000);
	}

	switch (i2c_status) {
	case HAL_OK:
		status = TCA9544A_OK;
		break;
	case HAL_BUSY:
		status = TCA9544A_BUSY;
		break;
	case HAL_TIMEOUT:
		status = TCA9544A_TIMEOUT;
		break;
	case HAL_ERROR:
	default:
		status = TCA9544A_ERROR;
		break;
	};

	return status;
}

TCA9544A_StatusTypeDef TCA9544A_ReadChannel(TCA9544A_HandleTypeDef *device) {
	HAL_StatusTypeDef i2c_status;
	TCA9544A_StatusTypeDef status;

	uint8_t register_read_buffer;
	i2c_status = HAL_I2C_Master_Receive(device->__hi2c, (TCA9544A_DevAddr | device->A2 << 3 | device->A1 << 2 | device->A0 << 1 | 1), &register_read_buffer, 1, 1000);

	switch (i2c_status) {
	case HAL_OK:
		status = TCA9544A_OK;
		break;
	case HAL_BUSY:
		status = TCA9544A_BUSY;
		break;
	case HAL_TIMEOUT:
		status = TCA9544A_TIMEOUT;
		break;
	case HAL_ERROR:
	default:
		status = TCA9544A_ERROR;
		break;
	};

	if (status == TCA9544A_OK) {
		if ((register_read_buffer & 0b00000100) == 0){
			device->current_channel = CHANNEL_NONE;
		} else {
			device->current_channel = (register_read_buffer & 0b00000011);
		}
	}

	return status;
};
