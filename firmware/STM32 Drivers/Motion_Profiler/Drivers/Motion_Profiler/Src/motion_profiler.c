/*
 * motion_profiler.c
 *
 *  Created on: May 24th, 2025
 *      Author: Adam
 */


#include "motion_profiler.h"

#include <math.h>

Motion_Profiler_StatusTypeDef Motion_Profiler_Init(Motion_Profiler_HandleTypeDef *hprofiler)
{
    if (hprofiler == NULL)
    {
        return MOTION_PROFILER_ERROR;
    }

    if (hprofiler->State == MOTION_PROFILER_STATE_READY)
    {
        // Peripheral is already initialized
        return MOTION_PROFILER_ERROR;
    }

    if (hprofiler->State == MOTION_PROFILER_STATE_BUSY)
    {
        return MOTION_PROFILER_BUSY;
    }

    if (hprofiler->State == MOTION_PROFILER_STATE_ERROR)
    {
        return MOTION_PROFILER_ERROR;
    }

    hprofiler->State = MOTION_PROFILER_STATE_READY;

    return MOTION_PROFILER_OK;

}


Motion_Profiler_StatusTypeDef Motion_Profiler_DeInit(Motion_Profiler_HandleTypeDef *hprofiler)
{
    if (hprofiler == NULL)
    {
        return MOTION_PROFILER_ERROR;
    }

    if (hprofiler->State == MOTION_PROFILER_STATE_RESET)
    {
        // Peripheral is not initialized
        return MOTION_PROFILER_ERROR;
    }

    if (hprofiler->State == MOTION_PROFILER_STATE_BUSY)
    {
        return MOTION_PROFILER_BUSY;
    }

    if (hprofiler->State == MOTION_PROFILER_STATE_ERROR)
    {
        return MOTION_PROFILER_ERROR;
    }

    hprofiler->State = MOTION_PROFILER_STATE_RESET;

    return MOTION_PROFILER_OK;

}

float MOTION_PROFILE_VELOCITY(uint32_t current_pos, uint32_t steps_to_move, float acceleration, float v_i, float v_max, uint32_t time_elapsed) {

	// Need to do some testing on the board, but start time might be 0 so we may not even need it!
	uint32_t set_point;
	float projected_time;
	float v_peak;
	float v_f;


	set_point = current_pos + steps_to_move;

	// If the set point is identical to the current position, exit the program

	if (set_point == current_pos) {
		return MOTION_PROFILER_BUSY;
	} else {

		// Calculation of the projected time

		projected_time = (float) MOTION_PROFILE_TIME(v_i, v_max, acceleration, steps_to_move);

		// Current time probably has to be passed in now that I think of it

		// todo: fix getting current time


		// Checking where we are in the motion based on the current time
		// todo: fix this after testing in the bay

		if (time_elapsed >= projected_time / 2) {
			acceleration = -fabsf(acceleration);
		} else {
			acceleration = fabsf(acceleration);
		}

		if (time_elapsed >= projected_time / 2 ) {

			v_peak = v_i + fabsf(acceleration * (projected_time/2));
			v_f = v_peak + acceleration*(time_elapsed-(projected_time/2));

		} else {
			v_f = v_i + acceleration*(time_elapsed);
		}

		// Returning the velocity

		if (fabsf(v_f) > v_max) {
			return v_max;
		} else if (v_f < v_max) {
			return v_f;
		}


	}

	return MOTION_PROFILER_ERROR;
}


float MOTION_PROFILE_TIME(float v_i, float v_max, float acceleration, uint32_t steps_to_move) {

	// Running time based calculations

	float t_max;
	float t_min;
	uint32_t steps_increase;
	uint32_t steps_decrease;
	uint32_t standard_steps;
	float t_level;
	float t_total;

	t_max = fabsf((v_max - v_i) / acceleration);
	t_min = fabsf(-v_max / acceleration);

	// Finding the steps of increase and decrease (which is needed later for total time)

	steps_increase = v_max*t_max - 0.5 * acceleration*(t_max*t_max);
	steps_decrease = v_max*t_min + 0.5*(-1*acceleration)*(t_min*t_min);
	standard_steps = steps_to_move - steps_increase - steps_decrease;

	// Finding the time at the level value

	t_level = (float) standard_steps / v_max;
	t_total = t_max + t_min + t_level;

	// Return the total time

	return t_total;




}


