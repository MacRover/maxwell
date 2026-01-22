/*
 * motion.c
 *
 *  Created on: Jun 7, 2025
 *      Author: Adam
 */

#include "motion.h"

Motion_Profile_HandleTypeDef motion_profile;


void MX_PROFILER_INIT() {

    motion_profile.STEPS_TO_MOVE = 90; // Can be arbitrarily changed later
    motion_profile.V_I = 0;
    motion_profile.V_MAX = 5;
    motion_profile.ACCELERATION = 2;
    motion_profile.CURRENT_POS = 0;
    motion_profile.SET_POINT = 0;
    motion_profile.TIME_ELAPSED = 0;
    motion_profile.VELOCITY = 0;
    motion_profile.MOVEMENT_STEPS = 0;
    motion_profile.TOTAL_STEPS = 0;

    if(Motion_Profile_Init(&motion_profile) != MOTION_PROFILE_OK) {
    	Error_Handler();
    }

 void MX_PROFILER_RESET() {

	motion_profile.V_I = 0;
	motion_profile.CURRENT_POS = 0;
	motion_profile.SET_POINT = 0;
	motion_profile.TIME_ELAPSED = 0;
	motion_profile.VELOCITY = 0;
	motion_profile.MOVEMENT_STEPS = 0;
	motion_profile.TOTAL_STEPS = 0;

    if(Motion_Profile_Init(&motion_profile) != MOTION_PROFILE_OK) {
    	Error_Handler();
    }

 }

}






