/*
 * TCA9548A.h
 *
 *  Created on: Nov 19, 2024
 *      Author: Adam and Vaibhav
 */

#ifndef TCA9544A_INC_TCA9544A_H_
#define TCA9544A_INC_TCA9544A_H_

typedef enum {
	CARD_NONE,
	CARD_0,
	CARD_1,
	CARD_2,
	CARD_3
} TCA9544A_CardSelect;

typedef struct {
	uint8_t A2;
	uint8_t A1;
	uint8_t A0;

	uint8_t INT0;
	uint8_t INT1;
	uint8_t INT2;
	uint8_t INT3;

	TCA9544A_CardSelect current_card;
} TCA9544A_DeviceType;

 ITCA9544A_DeviceType TCA9544A_Device;

void TCA9544A_Init(TCA9544A_DeviceType *device);

void TCA9544A_EnableCard(TCA9544A_DeviceType *device, TCA9544A_CardSelect card);

void TCA9544A_DisableAll(TCA9544A_DeviceType *device);

#endif /* TCA9544A_INC_TCA9544A_H_ */
