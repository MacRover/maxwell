/*
 * tca.c
 *
 *  Created on: Nov 27, 2024
 *      Author: Vaibhav & Adam
 */

#include "tca.h"

TCA9544A_HandleTypeDef tca;

void TCA_Init() {
	tca.__hi2c = &hi2c2;

	tca.A2 = 0;
	tca.A1 = 0;
	tca.A0 = 0;

	tca.current_channel = CHANNEL_NONE;

	if (TCA9544A_Init(&tca) != TCA9544A_OK) {
		Error_Handler();
	}
}


