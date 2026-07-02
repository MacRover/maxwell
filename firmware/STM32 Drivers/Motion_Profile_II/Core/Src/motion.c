/*
 * motion.c
 *
 *  Created on: Jun 7, 2025
 *      Author: Adam
 */

#include "motion.h"

Motion_Profile_HandleTypeDef motion_profile;


void MX_PROFILER_INIT(void) {

    motion_profile.STEPS_TO_MOVE = 90; // Can be arbitrarily changed later
    motion_profile.V_I = 0.0f;
    motion_profile.V_MAX = 5.0f;
    motion_profile.ACCELERATION = 2.0f;
    motion_profile.DIRECTION = 1;

    motion_profile.CURRENT_POS = 0;
    motion_profile.SET_POINT = 0;
    motion_profile.TIME_ELAPSED = 0.0f;
    motion_profile.VELOCITY = 0.0f;
    motion_profile.MOVEMENT_STEPS = 0;
    motion_profile.TOTAL_STEPS = 0;

    motion_profile.V_PEAK = 0.0f;
    motion_profile.T_INCREASING = 0.0f;
    motion_profile.T_DECREASING = 0.0f;
    motion_profile.T_LEVEL = 0.0f;
    motion_profile.T_TOTAL = 0.0f;

    if(Motion_Profile_Init(&motion_profile) != MOTION_PROFILE_OK) {
    	Error_Handler();
    }
}

 void MX_PROFILER_RESET(void) {

	motion_profile.CURRENT_POS = 0;
	motion_profile.SET_POINT = 0;
	motion_profile.TIME_ELAPSED = 0.0f;
	motion_profile.VELOCITY = 0.0f;
	motion_profile.MOVEMENT_STEPS = 0;
	motion_profile.TOTAL_STEPS = 0;

	motion_profile.V_PEAK = 0.0f;
	motion_profile.T_INCREASING = 0.0f;
	motion_profile.T_DECREASING = 0.0f;
	motion_profile.T_LEVEL = 0.0f;
	motion_profile.T_TOTAL = 0.0f;

	motion_profile.DIRECTION = 1;

    if(Motion_Profile_Init(&motion_profile) != MOTION_PROFILE_OK) {
    	Error_Handler();
    }

 }








