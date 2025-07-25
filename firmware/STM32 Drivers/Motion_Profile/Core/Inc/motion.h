/*
 * motion.h
 *
 *  Created on: Jun 7, 2025
 *      Author: zokur
 */

#ifndef INC_MOTION_H_
#define INC_MOTION_H_

#include "main.h"
#include "motion_profile.h"

// Again I hope we can get away without this'

extern Motion_Profile_HandleTypeDef motion_profile;

void Profiler_Init();
void Profiler_Reset();




#endif /* INC_MOTION_H_ */
