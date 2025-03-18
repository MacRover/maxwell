/*
 * rad_ntcb572.h
 *
 *  Created on: Mar 18, 2025
 *      Author: Ali
 */

#ifndef INC_RAD_NTCB572_H_
#define INC_RAD_NTCB572_H_

#include "main.h"
#include "ntcb572.h"

// Expose a global handle for the specific thermistor instance
extern NTC_HandleTypeDef rad_ntc1;

void MX_RAD_NTCB572_Init(void);

#endif

