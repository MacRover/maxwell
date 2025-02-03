#ifndef INC_INA_238_H_
#define INC_INA_238_H_

#include "main.h"
#include "i2c.h"
#include "stm32f1xx_ina238.h"

extern I2C_HandleTypeDef hi2c2;

extern INA_238_HandleTypeDef ina238_low_power_a;
extern INA_238_HandleTypeDef ina238_low_power_b;

extern INA_238_HandleTypeDef ina238_high_power;


void MX_INA_238_Init();

#endif