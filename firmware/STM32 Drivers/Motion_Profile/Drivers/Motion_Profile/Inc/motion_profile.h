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

typedef struct {
	uint32_t STEPS_TO_MOVE;
	uint32_t V_I;
	uint32_t V_MAX;
	uint32_t ACCELERATION;
	uint32_t CURRENT_POS;
	float TIME_ELAPSED;
	float VELOCITY;
	uint32_t MOVEMENT_STEPS;
	uint32_t TOTAL_STEPS;
} Motion_Profile_HandleTypeDef;

/**
 * @brief  Motion Profile Configuration Structure definition
 */



// TODO: Init/deinit properly

Motion_Profile_StatusTypeDef Motion_Profile_Init(Motion_Profile_HandleTypeDef *profile);
Motion_Profile_StatusTypeDef Motion_Profile_Reset(Motion_Profile_HandleTypeDef *profile);
float Motion_Profile_Time(Motion_Profile_HandleTypeDef *profile);
Motion_Profile_StateTypeDef Motion_Profile_Velocity(Motion_Profile_HandleTypeDef *profile, uint32_t start_time);

#endif /* MOTION_PROFILE_INC_MOTION_PROFILE_H_ */
