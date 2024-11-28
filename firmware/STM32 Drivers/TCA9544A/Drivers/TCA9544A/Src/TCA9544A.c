/*
 * TCA9548A.c
 *
 *  Created on: Nov 19, 2024
 *      Author: Adam and Vaibhav
 */

#include "TCA9544A.h"

void TCA9544A_Init(TCA9544A_DeviceType *device, I2C_HandleTypeDef *hi2c) {
	device->A2 = 0;
	device->A1 = 0;
	device->A0 = 0;

	device->address = 0b11100000;

	device->INT0 = 0;
	device->INT1 = 0;
	device->INT2 = 0;
	device->INT3 = 0;

	device->current_card = CARD_NONE;

	device->__hi2c = hi2c;
}

TCA9544A_StatusTypeDef TCA9544A_SelectCard(TCA9544A_DeviceType *device, TCA9544A_CardSelect card) {
	device->current_card = card;
	HAL_StatusTypeDef i2c_status;
	TCA9544A_StatusTypeDef status;

	if (device->current_card == CARD_NONE) {
		uint8_t register_write_buffer = 0b00000000;
		i2c_status = HAL_I2C_Master_Transmit(device->__hi2c, (device->address | device->A2 << 3 | device->A1 << 2 | device->A0 << 1 | 1), *register_write_buffer, 1, 1000);
	} else {
		uint8_t register_write_buffer = 0b00000100 | (device->current_card - 1);
		i2c_status = HAL_I2C_Master_Transmit(device->__hi2c, (device->address | device->A2 << 3 | device->A1 << 2 | device->A0 << 1 | 1), *register_write_buffer, 1, 1000);
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
