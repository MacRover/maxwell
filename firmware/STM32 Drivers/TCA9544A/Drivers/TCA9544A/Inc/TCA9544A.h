/*
 * TCA9548A.h
 *
 *  Created on: Nov 19, 2024
 *      Author: Adam and Vaibhav
 */

#ifndef TCA9544A_INC_TCA9544A_H_
#define TCA9544A_INC_TCA9544A_H_

#include "stm32f1xx.h"

#define TCA9544A_DevAddr 0b11100000

typedef enum {
	CHANNEL_0,
	CHANNEL_1,
	CHANNEL_2,
	CHANNEL_3,
	CHANNEL_NONE
} TCA9544A_ChannelSelect;

typedef enum {
	TCA9544A_OK,
	TCA9544A_ERROR,
	TCA9544A_BUSY,
	TCA9544A_TIMEOUT
} TCA9544A_StatusTypeDef;

typedef struct {
	uint8_t A2;
	uint8_t A1;
	uint8_t A0;

	TCA9544A_ChannelSelect current_channel;

	I2C_HandleTypeDef *__hi2c;
} TCA9544A_HandleTypeDef;

TCA9544A_StatusTypeDef TCA9544A_Init(TCA9544A_HandleTypeDef *device);

TCA9544A_StatusTypeDef TCA9544A_SelectChannel(TCA9544A_HandleTypeDef *device, TCA9544A_ChannelSelect channel);

TCA9544A_StatusTypeDef TCA9544A_ReadChannel(TCA9544A_HandleTypeDef *device);

#endif /* TCA9544A_INC_TCA9544A_H_ */
