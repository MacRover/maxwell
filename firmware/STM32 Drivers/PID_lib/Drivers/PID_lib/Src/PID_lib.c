/*
 * PID_lib.c
 *
 *  Created on: Jun 20, 2024
 *      Author: Ethan
 */

#include "PID_lib.h"

void PID_Init(PID_HandleTypeDef *PID)
{
    if (PID == NULL)
    {
        // TODO throw error
    }

    PID->__time = HAL_GetTick();

    PID->__error_old = *(PID->Init.feedback) - PID->__set_point;
    PID->__i_error = 0.0;

    // todo zero out error & other values
}

void PID_Update(PID_HandleTypeDef *PID)
{
    double feedback_value = *(PID->Init.feedback);
    uint32_t current_time = HAL_GetTick();
    double dt = (double) (current_time - PID->__time);

    PID->__time = current_time;
    PID->__error = PID->__set_point - feedback_value;

    double proportional = PID->Init.kp * PID->__error;

    PID->__i_error = PID->__i_error + (PID->__error * dt);
    double integral = PID->Init.ki * PID->__i_error;

    // todo remove this hack
    if (current_time == PID->__time)
    {
        dt = 1;
    }
    double derivative = PID->Init.kd * ((PID->__error - PID->__error_old) / dt);
    PID->__error_old = PID->__error;

    double output_raw = proportional + integral + derivative;

    if (output_raw > PID->Init.max_output_abs)
    {
        PID->output = PID->Init.max_output_abs;
        return;
    }
    if (output_raw < -1.0 * PID->Init.max_output_abs)
    {
        PID->output = -1.0 * PID->Init.max_output_abs;
        return;
    }
    PID->output = output_raw;
}

void PID_ChangeSetPoint(PID_HandleTypeDef *PID, double set_point)
{
    PID->__set_point = set_point;
    PID->__i_error = 0.0;
    PID->__error_old = *(PID->Init.feedback) - PID->__set_point;
}

//void PID_DeInit(PID_HandleTypeDef *PID){
//
//}

// todo add ability to update kp, ki, kd
// todo determine if this is a good way of formatting the library
// todo add return statuses and error codes
