/*
 * motion_profiler.c
 *
 *  Created on: December 12, 2024
 *      Author: Ishan
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

Motion_Profiler_StatusTypeDef Motion_Profiler_GenerateNewTrajectory(Motion_Profiler_HandleTypeDef *hprofiler, double current_location, double desired_setpoint)
{

    double error = fabs(desired_setpoint - current_location);

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
        //Already in a profile, use current velocity as min velocity

        if (hprofiler->Init.acceleration == 0)
        {
            hprofiler->blend_x_start = 0;
            hprofiler->blend_x_end = 0;
        }
        else
        {   
            hprofiler->blend_x_start = (pow(hprofiler->Init.max_velocity, 2) - pow(hprofiler->current_velocity, 2))/(2*hprofiler->Init.acceleration);
            hprofiler->blend_x_end = (pow(hprofiler->Init.max_velocity, 2) - pow(hprofiler->Init.min_velocity, 2))/(2*hprofiler->Init.acceleration);
        }

        hprofiler->current_min_velocity = hprofiler->current_velocity;


    }
    else
    {

        // Motion profiler is ready
        if (hprofiler->Init.acceleration == 0)
        {
            hprofiler->blend_x_start = 0;
            hprofiler->blend_x_end = 0;
        }
        else
        {   
            hprofiler->blend_x_start = (pow(hprofiler->Init.max_velocity, 2) - pow(hprofiler->Init.min_velocity, 2))/(2*hprofiler->Init.acceleration);
            hprofiler->blend_x_end = hprofiler->blend_x_start;
        }

        hprofiler->current_min_velocity = hprofiler->Init.min_velocity;

    }

    if ((current_location + hprofiler->blend_x_start) > (desired_setpoint - hprofiler->blend_x_end))
    {
        //Can't hit max velocity
        //Find point where ramp up meets ramp down, and then assign blendx from there

        double offset_point = error/2 - (pow(hprofiler->current_velocity, 2) - pow(hprofiler->Init.min_velocity, 2))/(2*hprofiler->Init.acceleration)/2;

        hprofiler->blend_x_start = offset_point;
        hprofiler->blend_x_end = error - offset_point;

    }

    hprofiler->start = current_location;
    hprofiler->setpoint = desired_setpoint;

    hprofiler->State = MOTION_PROFILER_STATE_BUSY;


    return MOTION_PROFILER_OK;

}

Motion_Profiler_StatusTypeDef Motion_Profiler_GetVelocity(Motion_Profiler_HandleTypeDef *hprofiler, double current_location)
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

    if (hprofiler->State == MOTION_PROFILER_STATE_READY)
    {
        //No profile generated
        return MOTION_PROFILER_NO_PROFILE;
    }


    //prioritize ramp down over ramp up

    if (fabs(hprofiler->setpoint - current_location) <= hprofiler->blend_x_end)
    {
        hprofiler->current_velocity = sqrt(pow(hprofiler->Init.min_velocity, 2) + 2 * hprofiler->Init.acceleration * fabs(hprofiler->setpoint - current_location));

        if ((fabs(hprofiler->setpoint - current_location) < hprofiler->Init.deadzone))
        {
            //Close enough to the target, suspend this profile
            hprofiler->State = MOTION_PROFILER_STATE_READY;

        }
    
    }
    else if (fabs(current_location - hprofiler->start) <= hprofiler->blend_x_start)
    {
        //still ramping up. Find target velocity based on current displacement
        //if accel is 0, blend_x is zero and this case is never triggered.

        hprofiler->current_velocity = sqrt(pow(hprofiler->current_min_velocity, 2) + 2 * hprofiler->Init.acceleration * fabs(current_location - hprofiler->start));

    }
    else
    {
        hprofiler->current_velocity = hprofiler->Init.max_velocity;
    }

    return MOTION_PROFILER_OK;

}
