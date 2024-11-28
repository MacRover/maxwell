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
	CARD_NONE,
	CARD_0,
	CARD_1,
	CARD_2,
	CARD_3
} TCA9544A_CardSelect;

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

	TCA9544A_CardSelect current_card;

	I2C_HandleTypeDef *__hi2c;
} TCA9544A_HandleTypeDef;

TCA9544A_StatusTypeDef TCA9544A_Init(TCA9544A_HandleTypeDef *device);

TCA9544A_StatusTypeDef TCA9544A_SelectCard(TCA9544A_HandleTypeDef *device, TCA9544A_CardSelect card);

#endif /* TCA9544A_INC_TCA9544A_H_ */
