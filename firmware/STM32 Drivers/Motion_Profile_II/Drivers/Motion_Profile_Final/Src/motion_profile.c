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

	if (profile == NULL) {
		return MOTION_PROFILE_ERROR;
	}

	return MOTION_PROFILE_OK;


}

Motion_Profile_StateTypeDef Motion_Profile_Velocity(Motion_Profile_HandleTypeDef *profile) {
	float velocity;


	float end_of_increase = profile->T_INCREASING;
	float end_of_level = profile->T_INCREASING + profile->T_LEVEL;
	float end_of_profile = profile->T_TOTAL;

	if (profile->SET_POINT == profile->CURRENT_POS) {
		profile->VELOCITY = 0.0f;
		return MOTION_PROFILE_STATE_DONE;
	}

	// Phase 1: Increasing

	if (profile->TIME_ELAPSED < end_of_increase) {
		float accel_rate = fabsf(profile->ACCELERATION);
		velocity = profile->V_I + accel_rate*profile->TIME_ELAPSED;
	}
	
	// Phase 2: Level Velocity

	else if (profile->TIME_ELAPSED < end_of_level) {
		velocity = profile->V_PEAK;
	}

	// Phase 3: Decreasing

	else if (profile->TIME_ELAPSED < end_of_profile) {
		float decel_rate = -fabsf(profile->ACCELERATION);
		// Need a deceleration time as well as this won't just work on it's own
		float decel_time = profile->TIME_ELAPSED - end_of_level;
		velocity = profile->V_PEAK + decel_rate*decel_time;
	}

	// Done case

	else {
		profile->VELOCITY = 0.0f;
		return MOTION_PROFILE_STATE_DONE;
	}

	// Now checking the safety limits

	if (isnan(velocity)) {
		profile->VELOCITY = 0.0f;
		return MOTION_PROFILE_STATE_ERROR;
	}


	if (velocity >= profile->V_MAX) {
		velocity = profile->V_MAX;
	} else if (velocity <= 0.0f) {
		velocity = 0.0f;
	}
	profile->VELOCITY = velocity * profile->DIRECTION;

	return MOTION_PROFILE_STATE_BUSY;







}

// Motion_Profile_StateTypeDef Motion_Profile_Velocity(Motion_Profile_HandleTypeDef *profile) {

// 	// Need to do some testing on the board, but start time might be 0 so we may not even need it!
// //	uint32_t current_time;
// 	float projected_time;
// 	float v_peak;
// 	float v_f;
// //	float time_elapsed;

// 	// If the set point is identical to the current position, exit the program

// 	if (profile->SET_POINT == profile->CURRENT_POS) {
// 		return MOTION_PROFILE_STATE_DONE;
// 	} else {

// 		// Calculation of the projected time

// 		projected_time = (float) Motion_Profile_Time(profile);

// 		// Elapsed time already calculated

// 		// Checking where we are in the motion based on the current time
// 		// todo: fix this after testing in the bay

// 		if (profile->TIME_ELAPSED >= projected_time / 2) {
// 			profile->ACCELERATION = -fabsf(profile->ACCELERATION);
// 		} else {
// 			profile->ACCELERATION = fabsf(profile->ACCELERATION);
// 		}

// 		if (profile->TIME_ELAPSED >= (projected_time / 2) ) {

// 			v_peak = profile->V_I + fabsf(profile->ACCELERATION * (projected_time/2));
// 			v_f = v_peak + profile->ACCELERATION*(profile->TIME_ELAPSED-(projected_time/2));

// 		} else {
// 			v_f = profile->V_I + profile->ACCELERATION*(profile->TIME_ELAPSED);
// 		}

// 		// Returning the velocity

// 		if (fabsf(v_f) > profile->V_MAX) {
// 			profile->VELOCITY = profile->V_MAX;

// 			// Checking to see if we are at the end of the profile (another exit failsafe)

// 			if (profile->TIME_ELAPSED >= (projected_time)) {
// 				return MOTION_PROFILE_STATE_DONE;
// 			} else {
// 				return MOTION_PROFILE_STATE_BUSY;
// 			}

// 		} else if (fabsf(v_f) <= profile->V_MAX) {
// 			profile->VELOCITY = v_f;

// 			// Checking to see if we are at the end of the profile

// 			if (profile->TIME_ELAPSED >= (projected_time)) {
// 				return MOTION_PROFILE_STATE_DONE;
// 			} else {
// 				return MOTION_PROFILE_STATE_BUSY;
// 			}

// 		} else {
// 			return MOTION_PROFILE_STATE_ERROR;
// 		}


// 	}

// 	return MOTION_PROFILE_STATE_ERROR;
// }


void Motion_Profile_Phases(Motion_Profile_HandleTypeDef *profile) {

	// Running time based calculations

	if (profile->STEPS_TO_MOVE >= 0) {
		profile->DIRECTION = 1;
	} else {
		profile->DIRECTION = -1;
	}

	float abs_steps = fabsf((float)profile->STEPS_TO_MOVE);

	uint32_t steps_increase;
	uint32_t steps_decrease;
	uint32_t standard_steps;

	profile->T_INCREASING = fabsf((profile->V_MAX - profile->V_I) / profile->ACCELERATION);
	profile->T_DECREASING = fabsf((profile->V_MAX / profile->ACCELERATION));

	// Finding the steps of increase and decrease (which is needed later for total time)

	// Stays the same
	steps_increase = profile->V_MAX*profile->T_INCREASING - 0.5f*profile->ACCELERATION*(profile->T_INCREASING*profile->T_INCREASING);

	// Rate of deceleration

	float deceleration_rate = -fabsf(profile->ACCELERATION);

	// Steps of decreasing

	// Stays the same
	steps_decrease = profile->V_MAX*profile->T_DECREASING + 0.5f*(deceleration_rate)*(profile->T_DECREASING*profile->T_DECREASING);


	if (steps_increase + steps_decrease > abs_steps) {

		// No standard steps at all
		standard_steps = 0;
		profile->T_LEVEL = 0;

		// Need to recalculate the peak velocity that the profile will go up to

		profile->V_PEAK = sqrtf(profile->ACCELERATION * abs_steps + 0.5f * (profile->V_I * profile->V_I));

		// Need to recalculate T_INCREASING and T_DECREASING

		profile->T_INCREASING = fabsf((profile->V_PEAK - profile->V_I) / profile->ACCELERATION);
		profile->T_DECREASING = fabsf((profile->V_PEAK / profile->ACCELERATION));

	} else {

		standard_steps = abs_steps - steps_increase - steps_decrease;
		profile->T_LEVEL = (float) standard_steps / profile->V_MAX;

		profile->V_PEAK = profile->V_MAX;
	}

	// Finding the time at the level value

	profile->T_TOTAL = profile->T_INCREASING + profile->T_DECREASING + profile->T_LEVEL;

	// Return the total time

}


