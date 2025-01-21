/*
 * PID_lib.h
 *
 *  Created on: Jun 20, 2024
 *      Author: Ethan
 */

#ifndef PID_LIB_INC_PID_LIB_H_
#define PID_LIB_INC_PID_LIB_H_

#include "stm32f1xx_hal.h"
#include <math.h>

typedef struct
{
    volatile double *feedback;
    double kp;
    double ki;
    double kd;
    double max_output_abs;
    double rollover_max;
    double min_output_abs;
} PID_InitTypeDef;

typedef struct
{
    PID_InitTypeDef Init;
    volatile double output;
    volatile double feedback_adj;
    volatile double __set_point;
    volatile double __error;
    volatile double __i_error;
    volatile int16_t __rollovers;
    volatile uint32_t __time_old;
    volatile double __error_old;
    volatile double __feedback_raw_old;
    volatile double __offset;
} PID_HandleTypeDef;

void PID_Init(PID_HandleTypeDef *PID);

void PID_Update(PID_HandleTypeDef *PID);
void PID_Update_RolloverCount(PID_HandleTypeDef *PID);
void PID_Update_BangBang(PID_HandleTypeDef *PID);


void PID_ChangeSetPoint(PID_HandleTypeDef *PID, double set_point);

void PID_SetZeroPoint(PID_HandleTypeDef *PID);
void PID_SetMaxPoint(PID_HandleTypeDef *PID, uint8_t max_rollovers);

void PID_ClearIError(PID_HandleTypeDef *PID);


#endif /* PID_LIB_INC_PID_LIB_H_ */
