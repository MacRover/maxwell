/*
 * motion_profiler.h
 *
 *  Created on: December 12, 2024
 *      Author: Ishan
 */


#ifndef MOTION_PROFILER_INC_MOTION_PROFILER_H_
#define MOTION_PROFILER_INC_MOTION_PROFILER_H_

#include <stdint.h>

/**
 * @brief  Motion Profiler Status structures definition
 */
typedef enum
{
    MOTION_PROFILER_OK = 0x00U,
    MOTION_PROFILER_ERROR = 0x01U,
    MOTION_PROFILER_BUSY = 0x02U,
    MOTION_PROFILER_NO_PROFILE = 0x03U
} Motion_Profiler_StatusTypeDef;

/**
 * @brief  Motion Profiler State structure definition
 */
typedef enum
{
    MOTION_PROFILER_STATE_RESET = 0x00U, /*!< Peripheral not Initialized                         */
    MOTION_PROFILER_STATE_READY = 0x01U, /*!< Peripheral Initialized and ready for use           */
    MOTION_PROFILER_STATE_BUSY = 0x02U, /*!< an internal process is ongoing                     */
    MOTION_PROFILER_STATE_ERROR = 0x06U, /*!< SPI error state                                    */
} Motion_Profiler_StateTypeDef;

/**
 * @brief  Motion Profiler Configuration Structure definition
 */


typedef struct 
{
    
    uint16_t max_velocity;
    uint16_t min_velocity;
    uint16_t acceleration;
    double deadzone;

} Motion_Profiler_InitTypeDef;

typedef struct
{
    Motion_Profiler_InitTypeDef Init;

    volatile Motion_Profiler_StateTypeDef State;

    volatile double setpoint;
    volatile double start;

    volatile uint16_t current_velocity;

    volatile double blend_x_start;
    volatile double blend_x_end;

    //tracks min velocity to use during ramp up in case we are mid profile and generate a new one
    volatile uint16_t current_min_velocity;

} Motion_Profiler_HandleTypeDef;


Motion_Profiler_StatusTypeDef Motion_Profiler_Init(Motion_Profiler_HandleTypeDef *hprofiler);
Motion_Profiler_StatusTypeDef Motion_Profiler_DeInit(Motion_Profiler_HandleTypeDef *hprofiler);

float time_calc(float v_i, float v_max, float acceleration, uint32_t steps_to_move);
float velocity_function(uint32_t current_pos, uint32_t steps_to_move, float acceleration, float v_i, float v_max, uint32_t start_time);

#endif /* MOTION_PROFILER_INC_MOTION_PROFILER_H_ */
