/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 MMRT.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "at24c04c.h"
#include "tmc_2590.h"
#include "as5048a.h"
#include "pid.h"
#include "queue.h"
#include "enc_dec_utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ROTATION_HIGH_UPPER_BOUND 365
#define ROTATION_LOW_UPPER_BOUND 30
#define ROTATION_HIGH_LOWER_BOUND 330
#define ROTATION_LOW_LOWER_BOUND -5
#define MIN_ROTATIONS 0
#define MAX_ROTATIONS 20


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
RAD_status_TypeDef rad_status;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

    /* USER CODE BEGIN 1 */



    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    uint8_t no_ccw_movement = 0;
    uint8_t no_cw_movement = 0;
    uint8_t condition = 0;

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_TIM2_Init();
    /* USER CODE BEGIN 2 */
    MX_AT24C04C_1_Init(); // config eeprom first so other init funcs can use it
    MX_TMC_2590_1_Init();
    MX_AS5048A_1_Init();
    MX_PID_1_Init();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    while (1)
    {
        if (!queue_empty(&can_message_queue_1))
        {
            RAD_CAN_Message_TypeDef *new_message =
                    (RAD_CAN_Message_TypeDef*) queue_front(
                            &can_message_queue_1);

            switch (new_message->command_id)
            {
            case SET_TARGET_ANGLE:
            {
                float new_setpoint = decode_float_big_endian(new_message->data);
                // todo get angle multiplier factor (ex for gearbox)
                PID_ChangeSetPoint(&pid_1, new_setpoint);
                break;
            }
            case SET_P_VALUE:
            {
                pid_1.Init.kp = (double) decode_float_big_endian(
                        new_message->data);
                AT24C04C_WriteData(&at24c04c_1, EEPROM_ADDR_P_VALUE,
                        (uint8_t*) &pid_1.Init.kp, sizeof(double));
                break;
            }
            case SET_I_VALUE:
            {
                pid_1.Init.ki = (double) decode_float_big_endian(
                        new_message->data);
                AT24C04C_WriteData(&at24c04c_1, EEPROM_ADDR_P_VALUE,
                        (uint8_t*) &pid_1.Init.ki, sizeof(double));
                break;
            }
            case SET_D_VALUE:
            {
                pid_1.Init.kd = (double) decode_float_big_endian(
                        new_message->data);
                AT24C04C_WriteData(&at24c04c_1, EEPROM_ADDR_P_VALUE,
                        (uint8_t*) &pid_1.Init.kd, sizeof(double));
                break;
            }
            case CALIBRATE_PID_POS_OFFSET:
            {
                // todo determine if ls is active high or low
//                while (!HAL_GPIO_ReadPin(LS_1_GPIO_Port, LS_1_Pin))
//                {
//                    // step back max amount
//                    while (TMC_2590_MoveSteps(&tmc_2590_1,
//                            -1.0 * tmc_2590_1.Init.max_steps) != TMC_2590_OK)
//                        ;
//                    // blocking wait for steps to complete before reading limit switch state again
//                    while (tmc_2590_1.State == TMC_2590_STATE_BUSY)
//                        ;
//                }
//                PID_SetZeroPoint(&pid_1);
//                PID_ChangeSetPoint(&pid_1, 0.0);
//                PID_Update(&pid_1);
                break;
            }
            case UPDATE_PID_POS_OFFSET:
            {
                PID_SetZeroPoint(&pid_1);
                PID_ChangeSetPoint(&pid_1, 0.0);
                PID_Update(&pid_1);
                break;
            }
            case SET_CAN_ID:
            {
                uint8_t new_id = new_message->data[0];
                AT24C04C_WriteData(&at24c04c_1, EEPROM_ADDR_CAN_ID, &new_id,
                        sizeof(uint8_t));

                // test code
                uint8_t eeprom_buff[1];
                AT24C04C_ReadData(&at24c04c_1, EEPROM_ADDR_CAN_ID, eeprom_buff,
                        sizeof(uint8_t));

                // will require a reset command to apply updated CAN ID
                break;
            }
            case RESET_BOARD:
            {
                NVIC_SystemReset(); // resets/reboots board
                break;
            }
            }
            free(new_message->data);
            queue_dequeue(&can_message_queue_1);

        }

        // limit switches are active LOW
        GPIO_PinState ls_1_state = HAL_GPIO_ReadPin(LS_1_GPIO_Port, LS_1_Pin);
        GPIO_PinState ls_2_state = HAL_GPIO_ReadPin(LS_2_GPIO_Port, LS_2_Pin);
        GPIO_PinState fsr_1_state = HAL_GPIO_ReadPin(FSR_1_GPIO_Port,
                FSR_1_Pin);
        GPIO_PinState fsr_2_state = HAL_GPIO_ReadPin(FSR_2_GPIO_Port,
                FSR_2_Pin);

        // todo stop motor from over-spinning but allow us to move away from limit switch if pressed
        // if (ls_1_state == 1 && ls_2_state == 1)
        // {

        // Initializing the encoder values
        float encoder_value;
        float last_encoder_value;
        uint8_t RAD_TYPE = 1; // Just doing this for now for analysis of behaviour

        // we set the past encoder value to the one we had before

        if (condition != 0) {
        	last_encoder_value = encoder_value;
        }


        // Reading the encoder value
        // 1. Ensure that the angle can be read and does not error
        // 2. Run the read angle command
        // 3. Capture the angle_double value in the encoder value local variable

        if (AS5048A_ReadAngle(&as5048a_1) == AS5048A_OK){
        	__read_angle_command(&as5048a_1);
        	encoder_value = as5048a_1.Angle_double;
        }

    	// If a rollover has been detected from 360 to 0, increment the rotations variable
    	// These are values that will need to be tested depending on read_eeprom

// This will not run on the first move (hence the condition variable)

        if (condition != 0) {


        	if ((pid_1.__rollovers != MAX_ROTATIONS) && (last_encoder_value >= ROTATION_HIGH_LOWER_BOUND && last_encoder_value <= ROTATION_HIGH_UPPER_BOUND && encoder_value >= ROTATION_LOW_LOWER_BOUND && encoder_value <= ROTATION_LOW_UPPER_BOUND)){
        		// If these conditions are true,

        		pid_1.__rollovers++;

        			// Now, check if a rollover has been detected from 360 to 0
        			// 360 <= current value <= 330
        			// 0 <= last value <= 30
        	} else if ((pid_1.__rollovers != MIN_ROTATIONS) && (encoder_value >= ROTATION_HIGH_LOWER_BOUND && encoder_value <= ROTATION_HIGH_UPPER_BOUND && last_encoder_value >= ROTATION_LOW_LOWER_BOUND && last_encoder_value <= ROTATION_LOW_UPPER_BOUND)){
        		pid_1.__rollovers--;

        		}

        }


        // For now, 1 is left and 0 is right

        if (RAD_TYPE == 1) {

        	// Conditions for if the limit switch has been pressed
        	// Allow for no more CW movement, and only CCW movement
        	if (ls_1_state == GPIO_PIN_RESET) {
        		pid_1.__rollovers = MAX_ROTATIONS;
        		no_ccw_movement = 0;
        		no_cw_movement = 1;

        	// Ensuring the limit switch cannot move clockwise if it is at the end of range
        	// Allow for no more cw movement, but only ccw movement
        	} else if (pid_1.__rollovers == MIN_ROTATIONS) {
        		no_cw_movement = 0;
        		no_ccw_movement = 1;

        	// Ensuring the ccw_movement and cw_movement variables are set back to 0 in all other instances
        	} else {
        		no_ccw_movement = 0;
        		no_cw_movement = 0;

        	}
        } else if (RAD_TYPE == 2) {
        	// Same as the code for rad type being a left motor, however, the rotation values are flipped

        	if (ls_1_state == GPIO_PIN_RESET) {
        		pid_1.__rollovers = MIN_ROTATIONS;
        		no_cw_movement = 0;
        		no_ccw_movement = 1;

        	// The other end for a right motor is then when the value is 20
        	} else if (pid_1.__rollovers == MAX_ROTATIONS) {
        		no_ccw_movement = 0;
        		no_cw_movement = 1;

        	} else {
        		no_ccw_movement = 0;
        		no_cw_movement = 0;
        	}
        }


        // Moving the wheel

       if (no_ccw_movement == 0 && ((int16_t) pid_1.output) >= 0) {
        	TMC_2590_MoveSteps(&tmc_2590_1, (int16_t) pid_1.output);
         } else if (no_cw_movement == 0 && (int16_t) pid_1.output <= 0) {
        	TMC_2590_MoveSteps(&tmc_2590_1, (int16_t) pid_1.output);
        }

        // Incrementing condition, since we now want to be running through all of the checking of the last encoder value

        if (condition == 0) {
        	condition = 1;
        }


        // }
        while (AS5048A_ReadAngle(&as5048a_1) != AS5048A_OK)
            ;
        PID_Update(&pid_1);

        if (HAL_GetTick() % 50 == 0)
        {
            rad_status.current_angle = (float) pid_1.feedback_adj;
//            rad_status.limit_switch_state = (uint8_t) ls_1_state;
//            rad_status.upper_bound_state = (uint8_t) ls_2_state; // TODO change this to the right value
            rad_status.ls_1 = ls_1_state;
            rad_status.ls_2 = ls_2_state;
            rad_status.fsr_1 = fsr_1_state;
            rad_status.fsr_2 = fsr_2_state;
            rad_status.kp = pid_1.Init.kp;
            rad_status.ki = pid_1.Init.ki;
            rad_status.kd = pid_1.Init.kd;
            MX_CAN_Broadcast_RAD_Status(&rad_can, rad_status);
        }

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct =
    { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct =
    { 0 };

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    TMC_2590_TIM_PWM_PulseFinishedCallback(&tmc_2590_1, htim);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
