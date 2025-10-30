/*
 * tca.h
 *
 * 	Created on: Nov 27, 2024
 * 		Author: Vaibhav and Adam
 */

#ifndef INC_TCA_H_
#define INC_TCA_H_

#include "main.h"
#include "i2c.h"
#include "TCA9544A.h"

extern I2C_HandleTypeDef hi2c2;

extern TCA9544A_HandleTypeDef tca;

void TCA_Init();

#endif
