/*
 * motion.c
 *
 *  Created on: May 24, 2025
 *      Author: Adam
 */

#include "motion.h"

Motion_Profiler_HandleTypeDef hprofiler;


void MOTION_Init(){
	if (Motion_Profiler_Init(&hprofiler) != MOTION_PROFILER_OK) {
		Error_Handler();
	}
}




