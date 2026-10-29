/*
 * tca.c
 *
 *  Created on: Oct 18, 2026
 *      Author: Manil Ferr
 */


#include "tca.h"

TCA9544A_HandleTypeDef tca;

void MX_TCA_Init() {
	tca.__hi2c = &hi2c2;

	tca.A2 = 0;
	tca.A1 = 0;
	tca.A0 = 0;

	tca.current_channel = CHANNEL_NONE;

	if (TCA9544A_Init(&tca) != TCA9544A_OK) {
		Error_Handler();
	}
}
