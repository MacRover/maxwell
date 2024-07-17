/*
 * pid.c
 *
 *  Created on: Jun 23, 2024
 *      Author: Ethan
 */

#include "pid.h"

#include "as5048a.h"

PID_HandleTypeDef pid_1;

void MX_PID_1_Init(void)
{
    // todo convert the angle into a double
    pid_1.Init.feedback = &(as5048a_1.Angle_double);
    pid_1.Init.kp = 1;
    pid_1.Init.ki = 0;
    pid_1.Init.kd = 0;
    pid_1.Init.max_output_abs = 1000;

    PID_Init(&pid_1);

//    if (PID_Init(&pid_1) != PID_OK)
//    {
//        Error_Handler();
//    }
}

