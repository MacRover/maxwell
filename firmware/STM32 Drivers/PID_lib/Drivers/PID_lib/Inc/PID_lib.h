/*
 * PID_lib.h
 *
 *  Created on: Jun 20, 2024
 *      Author: Ethan
 */

#ifndef PID_LIB_INC_PID_LIB_H_
#define PID_LIB_INC_PID_LIB_H_

#include "stm32f1xx_hal.h"

typedef struct
{
    volatile double *feedback;
    double kp;
    double ki;
    double kd;
    double max_output_abs;
} PID_InitTypeDef;

typedef struct
{
    PID_InitTypeDef Init;
    volatile uint32_t __time;
    volatile double __set_point;
    volatile double __error;
    volatile double __error_old;
    volatile double __i_error;
    volatile double output;
} PID_HandleTypeDef;

void PID_Init(PID_HandleTypeDef *PID);

void PID_Update(PID_HandleTypeDef *PID);

void PID_ChangeSetPoint(PID_HandleTypeDef *PID, double set_point);

#endif /* PID_LIB_INC_PID_LIB_H_ */
