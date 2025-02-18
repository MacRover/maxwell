/*
 * tmp.h
 *
 *  Created on: Nov 22, 2024
 *      Author: John
 */

#ifndef INC_TMP_H_
#define INC_TMP_H_

#include "main.h"
#include "i2c.h"
#include "tmp_100.h"

extern I2C_HandleTypeDef hi2c2;

extern TMP_100_HandleTypeDef h_tmp_100_a;
extern TMP_100_HandleTypeDef h_tmp_100_b;

void MX_TMP_100_Init();


#endif /* INC_TMP_H_ */
