/*
 * motionprofile.c
 *
 *  Created on: Dec 13, 2024
 *      Author: Ishan
 */

#include "motionprofile.h"

Motion_Profiler_HandleTypeDef motion_profile_1;

void MX_Motion_Profile_1_Init(void)
{
    motion_profile_1.Init.max_velocity = 2000;
    motion_profile_1.Init.min_velocity = 500;

    motion_profile_1.Init.acceleration = 5000; //1 second to ramp up/down
    motion_profile_1.Init.deadzone = 10;

   
    Motion_Profiler_Init(&motion_profile_1);


}

