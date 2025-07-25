/*
 * motion_profile.c
 *
 *  Created on: May 24th, 2025
 *      Author: Adam
 */


#include "motion_profile.h"
#include <math.h>


Motion_Profile_StatusTypeDef Motion_Profile_Init(Motion_Profile_HandleTypeDef *profile) {

	// Variable initializations for driver

    profile->STEPS_TO_MOVE = 90; // Can be arbitrarily changed later
    profile->V_I = 0;
    profile->V_MAX = 5;
    profile->ACCELERATION = 2;
    profile->CURRENT_POS = 0;
    profile->TIME_ELAPSED = 0;
    profile->VELOCITY = 0;
    profile->MOVEMENT_STEPS = 0;
    profile->TOTAL_STEPS = 0;

    return MOTION_PROFILE_OK;
}

Motion_Profile_StatusTypeDef Motion_Profile_Reset(Motion_Profile_HandleTypeDef *profile) {
	profile->CURRENT_POS = 0;
	profile->TIME_ELAPSED = 0;
	profile->VELOCITY = 0;
	profile->MOVEMENT_STEPS = 0;
	profile->TOTAL_STEPS = 0;

	return MOTION_PROFILE_OK;
}


Motion_Profile_StateTypeDef Motion_Profile_Velocity(Motion_Profile_HandleTypeDef *profile, uint32_t start_time) {

	// Need to do some testing on the board, but start time might be 0 so we may not even need it!
	uint32_t set_point;
	uint32_t current_time;
	float projected_time;
	float v_peak;
	float v_f;
	float time_elapsed;

	// If the set point is identical to the current position, exit the program

	if (set_point == profile->CURRENT_POS) {
		return MOTION_PROFILE_STATE_READY;
	} else {

		// Calculation of the projected time

		projected_time = (float) Motion_Profile_Time(profile);

		// Get the current time

		current_time = HAL_GetTick();


		profile->TIME_ELAPSED = (float) (current_time - start_time) / 1000;




		// Checking where we are in the motion based on the current time
		// todo: fix this after testing in the bay

		if (profile->TIME_ELAPSED >= projected_time / 2) {
			profile->ACCELERATION = -fabsf(profile->ACCELERATION);
		} else {
			profile->ACCELERATION = fabsf(profile->ACCELERATION);
		}

		if (profile->TIME_ELAPSED >= (projected_time / 2) ) {

			v_peak = profile->V_I + fabsf(profile->ACCELERATION * (projected_time/2));
			v_f = v_peak + profile->ACCELERATION*(profile->TIME_ELAPSED-(projected_time/2));

		} else {
			v_f = profile->V_I + profile->ACCELERATION*(profile->ACCELERATION);
		}

		// Returning the velocity

		if (fabsf(v_f) > profile->V_MAX) {
			profile->VELOCITY = profile->V_MAX;
			return MOTION_PROFILE_STATE_BUSY;
		} else if (fabsf(v_f) < profile->V_MAX) {
			profile->VELOCITY = v_f;
			return MOTION_PROFILE_STATE_BUSY;
		} else {
			return MOTION_PROFILE_STATE_ERROR;
		}


	}

	return MOTION_PROFILE_ERROR;
}


float Motion_Profile_Time(Motion_Profile_HandleTypeDef *profile) {

	// Running time based calculations

	float t_max;
	float t_min;
	uint32_t steps_increase;
	uint32_t steps_decrease;
	uint32_t standard_steps;
	float t_level;
	float t_total;

	t_max = fabsf((profile->V_MAX - profile->V_I) / profile->ACCELERATION);
	t_min = fabsf(-profile->V_MAX / profile->ACCELERATION);

	// Finding the steps of increase and decrease (which is needed later for total time)

	steps_increase = profile->V_MAX*t_max - 0.5*profile->ACCELERATION*(t_max*t_max);
	steps_decrease = profile->V_MAX*t_min + 0.5*(-1*profile->ACCELERATION)*(t_min*t_min);
	standard_steps = profile->STEPS_TO_MOVE - steps_increase - steps_decrease;

	// Finding the time at the level value

	t_level = (float) standard_steps / profile->V_MAX;
	t_total = t_max + t_min + t_level;

	// Return the total time

	return t_total;

}


