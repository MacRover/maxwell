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
    pid_1.Init.feedback = &(as5048a_1.Angle_double);
    // todo pull from eeprom
    pid_1.Init.kp = 0.06;
    pid_1.Init.ki = 0.0001;
    pid_1.Init.kd = 0.0;
    pid_1.Init.max_output_abs = 1000.0;
    pid_1.Init.rollover_max = 360.0;

    PID_Init(&pid_1);

//    if (PID_Init(&pid_1) != PID_OK)
//    {
//        Error_Handler();
//    }

    // set startup as zero point
    PID_SetZeroPoint(&pid_1);
    PID_ChangeSetPoint(&pid_1, 0.0);
    PID_Update(&pid_1);
}

