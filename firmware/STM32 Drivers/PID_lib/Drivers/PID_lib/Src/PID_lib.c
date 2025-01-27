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

    PID->__time_old = HAL_GetTick();

    PID->__error_old = *(PID->Init.feedback) - PID->__set_point;
    PID->__i_error = 0.0;

    PID->__rollovers = 0;
    PID->__feedback_raw_old = PID->Init.rollover_max / 2.0; // half way mark
    PID->__offset = 0.0;
    // todo zero out error & other values
}

void PID_Update_BangBang(PID_HandleTypeDef *PID)
{
    double feedback_raw = *(PID->Init.feedback);

    // adjust rollover count
    if (feedback_raw < PID->Init.rollover_max * 0.1
            && PID->__feedback_raw_old > PID->Init.rollover_max * 0.9)
    {
        PID->__rollovers++;
    }

    if (feedback_raw > PID->Init.rollover_max * 0.9
            && PID->__feedback_raw_old < PID->Init.rollover_max * 0.1)
    {
        PID->__rollovers--;
    }

    PID->feedback_adj = feedback_raw + PID->__rollovers * PID->Init.rollover_max
            + PID->__offset;

    //Number of steps we are away from our target
    PID->__error = PID->__set_point - PID->feedback_adj;

    // set old values for next update
    PID->__error_old = PID->__error;
    PID->__feedback_raw_old = feedback_raw;

    PID->output = PID->__error;

    // fit output to bounds
    if (PID->__error > PID->Init.max_output_abs)
    {
        PID->output = PID->Init.max_output_abs;
    }

    else if (PID->__error < -1.0 * PID->Init.max_output_abs)
    {
        PID->output = -1.0 * PID->Init.max_output_abs;
    }

    //Apply deadzone
    else if (fabs(PID->__error) < PID->Init.min_output_abs)
	{
		PID->output = 0;
	}


    return;

}

void PID_Update_RolloverCount(PID_HandleTypeDef *PID)
{
    double feedback_raw = *(PID->Init.feedback);

    // adjust rollover count
    if (feedback_raw < PID->Init.rollover_max * 0.1
            && PID->__feedback_raw_old > PID->Init.rollover_max * 0.9)
    {
        PID->__rollovers++;
    }

    if (feedback_raw > PID->Init.rollover_max * 0.9
            && PID->__feedback_raw_old < PID->Init.rollover_max * 0.1)
    {
        PID->__rollovers--;
    }

    PID->__feedback_raw_old = feedback_raw;

    PID->feedback_adj = feedback_raw + PID->__rollovers * PID->Init.rollover_max
            + PID->__offset;
}

void PID_Update(PID_HandleTypeDef *PID)
{
    double feedback_raw = *(PID->Init.feedback);
    uint32_t current_time = HAL_GetTick();

    // adjust rollover count
    if (feedback_raw < PID->Init.rollover_max * 0.1
            && PID->__feedback_raw_old > PID->Init.rollover_max * 0.9)
    {
        PID->__rollovers++;
    }

    if (feedback_raw > PID->Init.rollover_max * 0.9
            && PID->__feedback_raw_old < PID->Init.rollover_max * 0.1)
    {
        PID->__rollovers--;
    }

    PID->feedback_adj = feedback_raw + PID->__rollovers * PID->Init.rollover_max
            + PID->__offset;
    double dt = (double) (current_time - PID->__time_old);

    PID->__error = PID->__set_point - PID->feedback_adj;


    PID->__i_error = PID->__i_error + (PID->__error * dt);

    double integral = PID->__i_error;
    double derivative =
            (PID->__time_old == current_time) ?
                    0 : (PID->__error - PID->__error_old) / dt; // avoid divide by zero errors
    double output_raw = (PID->Init.kp * PID->__error)
            + (PID->Init.ki * integral) + (PID->Init.kd * derivative); // P, I, D

    // set old values for next update
    PID->__error_old = PID->__error;
    PID->__time_old = current_time;
    PID->__feedback_raw_old = feedback_raw;

    // fit output to bounds
    if (output_raw > PID->Init.max_output_abs)
    {
        PID->output = PID->Init.max_output_abs;
        return;
    }

    else if (output_raw < -1.0 * PID->Init.max_output_abs)
    {
        PID->output = -1.0 * PID->Init.max_output_abs;
        return;
    }

    else if (fabs(output_raw) < PID->Init.min_output_abs)
	{
		PID->output = 0;
        PID->__i_error = 0.0;
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

void PID_SetZeroPoint(PID_HandleTypeDef *PID)
{
    PID->__offset = -1.0 * PID->__feedback_raw_old;
    PID->__rollovers = 0;
}

void PID_SetMaxPoint(PID_HandleTypeDef *PID, uint8_t max_rollovers)
{
    PID->__offset = -1.0 * PID->__feedback_raw_old;
    PID->__rollovers = max_rollovers;
}

void PID_ClearIError(PID_HandleTypeDef *PID)
{
    PID->__i_error = 0;
}

// todo add return statuses and error codes
