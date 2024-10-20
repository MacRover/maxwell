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
        TMC_2590_MoveSteps(&tmc_2590_1, (int16_t) pid_1.output);
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
