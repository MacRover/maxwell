/*
 * pid.c
 *
 *  Created on: Jun 23, 2024
 *      Author: Ethan
 */

#include "pid.h"
#include "as5048a.h"
#include "at24c04c.h"
#include "enc_dec_utils.h"

#include "main.h"

PID_HandleTypeDef pid_1;

void MX_PID_1_Init(void)
{
    pid_1.Init.feedback = &(as5048a_1.Angle_double);

    //default params. Will be used if EEPROM cannot be read
    pid_1.Init.kp = rad_params.P;
    pid_1.Init.ki = rad_params.I;
    pid_1.Init.kd = rad_params.D;

    pid_1.Init.max_output_abs = 50.0;
    pid_1.Init.min_output_abs = 7; //7*2 = 14 steps * (6 deg/200 steps) ~ 0.45 deg
    pid_1.Init.rollover_max = 360.0;
    pid_1.Init.error_threshold = 20; //20 steps * (6 deg/200 steps) = 0.6 deg
    PID_Init(&pid_1);

//    if (PID_Init(&pid_1) != PID_OK)
//    {
//        Error_Handler();
//    }

    PID_Update(&pid_1);
    PID_Update(&pid_1);
    PID_Update(&pid_1);
    // set startup as zero point
    PID_SetZeroPoint(&pid_1);
    PID_ChangeSetPoint(&pid_1, 0.0);
    PID_Update(&pid_1);
}

