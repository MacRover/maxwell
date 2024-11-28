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
#include "tmp_1075.h"

extern I2C_HandleTypeDef hi2c2;

extern TMP_1075_HandleTypeDef h_tmp_1075;

void MX_TMP_1075_Init();


#endif /* INC_TMP_H_ */
