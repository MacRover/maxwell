// Code goes here!!
/*
 * motion_profile.h
 *
 *  Created on: May 24th, 2024
 *      Author: Adam
 */


#ifndef MOTION_PROFILE_INC_MOTION_PROFILE_H_
#define MOTION_PROFILE_INC_MOTION_PROFILE_H_

#include <stdint.h>

/**
 * @brief  Motion Profile Status structures definition
 */
typedef enum
{
    MOTION_PROFILE_OK = 0x00U,
    MOTION_PROFILE_ERROR = 0x01U,
    MOTION_PROFILE_BUSY = 0x02U,
    MOTION_PROFILE_NO_PROFILE = 0x03U
} Motion_Profile_StatusTypeDef;

/**
 * @brief  Motion Profiler State structure definition
 */
typedef enum
{
    MOTION_PROFILE_STATE_RESET = 0x00U, /*!< Peripheral not Initialized                         */
    MOTION_PROFILE_STATE_READY = 0x01U, /*!< Peripheral Initialized and ready for use           */
    MOTION_PROFILE_STATE_BUSY = 0x02U, /*!< an internal process is ongoing                     */
    MOTION_PROFILE_STATE_ERROR = 0x06U, /*!< SPI error state                                    */
} Motion_Profile_StateTypeDef;

/**
 * @brief  Motion Profile Configuration Structure definition
 */



// TODO: Init/deinit properly

float Motion_Profile_Time(float v_i, float v_max, float acceleration, uint32_t steps_to_move);
float Motion_Profile_Velocity(uint32_t current_pos, uint32_t steps_to_move, float acceleration, float v_i, float v_max, uint32_t start_time);

#endif /* MOTION_PROFILE_INC_MOTION_PROFILE_H_ */
