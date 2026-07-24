/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Motion Profile Testbench
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tmc_2590.h"
#include "queue.h"
#include "enc_dec_utils.h"
#include "motion.h"
#include <math.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Kept for CAN and internal driver references
#define MOTOR_GEARING 1
#define STEPS_PER_REVOLUTION 200
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
RAD_STATUS_TypeDef rad_status;
RAD_PARAMS_TypeDef rad_params;

uint8_t ESTOP = 0;
uint8_t DISABLED = 0;

uint32_t profile_start_time = 0;
uint32_t prev_ms = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MOTION_PROFILE_Set_Speed(uint16_t velocity);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

    // SET DEFAULT VALUES (Stripped of PID/EEPROM defaults)
    rad_params.RAD_ID = 0xF0;
    rad_params.RAD_TYPE = RAD_TYPE_UNDEFINED;
    rad_params.STEPPER_SPEED = 1000;
    rad_params.ODOM_INTERVAL = 20; // 50hz, or 20ms
    rad_params.HEALTH_INTERVAL = 1000; // every second
    
    rad_params.CHOPCONF_CHM = 0b0;
    rad_params.CHOPCONF_HDEC = 0b00;
    rad_params.CHOPCONF_HEND = 0b0100;
    rad_params.CHOPCONF_HSTRT = 0b110;
    rad_params.CHOPCONF_RNDTF = 0b0;
    rad_params.CHOPCONF_TBL = 0b10;
    rad_params.CHOPCONF_TOFF = 0b100;

    rad_params.DRVCONF_DIS_S2G = 0b0;
    rad_params.DRVCONF_EN_PFD = 0b1;
    rad_params.DRVCONF_EN_S2VS = 0b1;
    rad_params.DRVCONF_OTSENS = 0b0;
    rad_params.DRVCONF_RDSEL = 0b11;
    rad_params.DRVCONF_SDOFF = 0b0;
    rad_params.DRVCONF_SHRTSENS = 0b1;
    rad_params.DRVCONF_SLP = 0b11110;
    rad_params.DRVCONF_TS2G = 0b00;
    rad_params.DRVCONF_TST = 0b0;
    rad_params.DRVCONF_VSENSE = 0b0;

    rad_params.DRVCTRL_DEDGE = 0b0;
    rad_params.DRVCTRL_INTPOL = 0b1;
    rad_params.DRVCTRL_MRES = 0b1000;

    rad_params.SGCSCONF_CS = 5;
    rad_params.SGCSCONF_SFILT = 0b0;
    rad_params.SGCSCONF_SGT = 0b0000010;

    rad_params.SMARTEN_SEDN = 0b00;
    rad_params.SMARTEN_SEIMIN = 0b0;
    rad_params.SMARTEN_SEMAX = 0b0000;
    rad_params.SMARTEN_SEMIN = 0b0000;
    rad_params.SMARTEN_SEUP = 0b00;

    rad_params.SW_STOP_ENABLED = 0;
    rad_params.WATCH_DOG_ENABLED = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */

    rad_status.flags = (rad_params.SW_STOP_ENABLED) | (rad_params.WATCH_DOG_ENABLED);

    // Initialize only the required drivers for the testbench
    MX_TMC_2590_1_Init();
    MX_PROFILER_INIT();

    // Default TMC Settings (Kept from your original init)
    tmc_2590_1.Init.inverted = 0;

    MX_CAN_UpdateIdAndFilters(&rad_can);

    uint32_t arr = HAL_TIM_CalculateAutoReload(tmc_2590_1.Init.STEP_Tim, rad_params.STEPPER_SPEED);
    TMC_2590_SetTimAutoReload(&tmc_2590_1, arr);

    MX_CAN_Broadcast_Health_Message(&rad_can, rad_status);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // TODO YOUR TRASH THIS IS WRONG - THIS IS SO WRONG WITH RAD STATE
    // THIS IS ACTUALLY FINE

    static enum 
    {
        RAD_STATE_INIT = 0,
        RAD_STATE_IDLE,
        RAD_STATE_PROFILE_CONTROL
    } rad_state = RAD_STATE_INIT;

    while (1)
    {
        // ------------------------------------------------------
        // 1. GLOBAL CAN MESSAGES
        // ------------------------------------------------------
        if (!queue_empty(&can_message_queue_global))
        {
            RAD_CAN_Message_TypeDef *new_message = (RAD_CAN_Message_TypeDef*) queue_front(&can_message_queue_global);

            switch ((int)(new_message->command_id))
            {
                case ESTOP_MESSAGE:
                    ESTOP = 1;
                    break;
                case DISABLE_MESSAGE:
                    DISABLED = 1;
                    break;
                case ENABLE_MESSAGE:
                    rad_state = RAD_STATE_IDLE;
                    DISABLED = 0;
                    ESTOP = 0;
                    break;
                case HEALTH_STATUS_PING:
                    MX_CAN_Broadcast_Health_Message(&rad_can, rad_status);
                    break;
                default:
                    break;
            }

            free(new_message->data);
            queue_dequeue(&can_message_queue_global);
        }
        // ------------------------------------------------------
        // 2. RAD CAN MESSAGES (Stripped of PID/EEPROM)
        // ------------------------------------------------------
        else if (!queue_empty(&can_message_queue_rad))
        {
            RAD_CAN_Message_TypeDef *new_message = (RAD_CAN_Message_TypeDef*) queue_front(&can_message_queue_rad);

            switch ((int)(new_message->command_id))
            {
                case PULSE_STEPPER:
                {

                    // REVIEW THIS
                    float pulses = decode_float_big_endian(new_message->data);
                    int32_t target_steps = (int32_t)pulses;

                    // Cap steps using TMC driver limits
                    if (target_steps > tmc_2590_1.Init.max_steps)
                        target_steps = tmc_2590_1.Init.max_steps;
                    else if (target_steps < -1*(tmc_2590_1.Init.max_steps))
                        target_steps = -1*tmc_2590_1.Init.max_steps;

                    // Setup the Math Profile
                    motion_profile.STEPS_TO_MOVE = target_steps;
                    Motion_Profile_Phases(&motion_profile);

                    // Setup the Hardware (Using your existing wrapper)
                    if (TMC_2590_CheckState(&tmc_2590_1) == TMC_2590_BUSY) {
                        TMC_2590_Stop(&tmc_2590_1);
                    }
                    rad_status.TMC_STATUS = TMC_2590_MoveSteps(&tmc_2590_1, target_steps);

                    // Start the clock and enter profiling state
                    profile_start_time = HAL_GetTick();
                    rad_state = RAD_STATE_PROFILE_CONTROL;
                    break;
                }
                case SET_STEPPER_SPEED:
                {
                    uint32_t arr = HAL_TIM_CalculateAutoReload(tmc_2590_1.Init.STEP_Tim, decode_uint32_big_endian(new_message->data));
                    TMC_2590_SetTimAutoReload(&tmc_2590_1, arr);
                    rad_params.STEPPER_SPEED = HAL_TIM_CalculateFrequency(tmc_2590_1.Init.STEP_Tim);
                    break;
                }
                case GET_STEPPER_SPEED:
                    MX_CAN_Broadcast_Uint32_Data(&rad_can, rad_params.STEPPER_SPEED, GET_STEPPER_SPEED);
                    break;

                case SET_RAD_FLAGS:
                {
                	uint8_t flags = new_message->data[0];
                	rad_params.SW_STOP_ENABLED = flags & (1 << 0);
                	rad_params.WATCH_DOG_ENABLED = flags & (1 << 1);
                	rad_status.flags = (rad_params.SW_STOP_ENABLED) | (rad_params.WATCH_DOG_ENABLED);
                	break;
                }
                case GET_RAD_FLAGS:
                	MX_CAN_Broadcast_Uint8_Data(&rad_can, rad_status.flags, GET_RAD_FLAGS);
                	break;

                case REBOOT:
                	HAL_NVIC_SystemReset();
                    break;
                case ASSIGN_DEVICE_ID:
                    rad_can.id = new_message->data[0];
                    rad_params.RAD_ID = rad_can.id;
                    MX_CAN_UpdateIdAndFilters(&rad_can);
                    break;

                case SET_VMAX:
                    motion_profile.V_MAX = decode_float_big_endian(new_message->data);
                    break;

                case GET_VMAX:
                    MX_CAN_Broadcast_Float_Data(&rad_can, motion_profile.V_MAX, GET_VMAX);
                    break;

                case SET_ACCELERATION:
                    motion_profile.ACCELERATION = decode_float_big_endian(new_message->data);
                    break;

                case GET_ACCELERATION:
                    MX_CAN_Broadcast_Float_Data(&rad_can, motion_profile.ACCELERATION, GET_ACCELERATION);
                    break;
                
                case SET_STEPS_TO_MOVE:
                    motion_profile.STEPS_TO_MOVE = decode_int32_big_endian(new_message->data);
                    break;

                case GET_STEPS_TO_MOVE:
                    MX_CAN_Broadcast_Int32_Data(&rad_can, motion_profile.STEPS_TO_MOVE, GET_STEPS_TO_MOVE);
                    break;

                // (Omitted the extensive TMC SPI tuning cases for brevity, but they can remain here untouched if needed for live testing)
                default:
                    break;
            }
            free(new_message->data);
            queue_dequeue(&can_message_queue_rad);
        }

        // ------------------------------------------------------
        // 3. SAFETY CHECKS
        // ------------------------------------------------------
        if (ESTOP || DISABLED)
        {
        	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
            TMC_2590_Stop(&tmc_2590_1);
            if (ESTOP) break;
            else continue;
        }
        else
        {
        	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
        }

        // ------------------------------------------------------
        // 4. MAIN STATE MACHINE
        // ------------------------------------------------------
        switch (rad_state)
        {
            case RAD_STATE_INIT:
                rad_state = RAD_STATE_IDLE;
                break;

            case RAD_STATE_IDLE:
                // Motor is stationary, waiting for PULSE_STEPPER
                break;

            case RAD_STATE_PROFILE_CONTROL:

            // REVIEW THIS SOME MORE
            {
                // Calculate elapsed time in seconds
                motion_profile.TIME_ELAPSED = (float)(HAL_GetTick() - profile_start_time) / 1000.0f;

                // Get target velocity for this exact millisecond
                Motion_Profile_StateTypeDef prof_state = Motion_Profile_Velocity(&motion_profile);

                if (prof_state == MOTION_PROFILE_STATE_BUSY)
                {
                    uint16_t target_speed = (uint16_t)fabsf(motion_profile.VELOCITY);
                    
                    if (target_speed > 0) {
                        // Dynamically adjust the timer ARR while MoveSteps runs in the background
                        MOTION_PROFILE_Set_Speed(target_speed);
                    }
                }
                else if (prof_state == MOTION_PROFILE_STATE_DONE)
                {
                    // Math confirms profile is complete
                    TMC_2590_Stop(&tmc_2590_1);
                    MX_PROFILER_RESET();
                    rad_state = RAD_STATE_IDLE;
                }
                break;
            }
            default:
                rad_state = RAD_STATE_INIT;
                break;
        }

        // ------------------------------------------------------
        // 5. TELEMETRY / GRAPHING
        // ------------------------------------------------------
        if ((rad_params.ODOM_INTERVAL != 0) && (HAL_GetTick() % rad_params.ODOM_INTERVAL == 0))
        {
            // Transmit the real-time calculated velocity over CAN for your SSH graph.
            // Repurposing the 'current_angle' variable specifically for this testbench visualization.
            rad_status.current_angle = (double)motion_profile.VELOCITY;
            MX_CAN_Broadcast_Odometry_Message(&rad_can, rad_status);
        }

        if ((rad_params.HEALTH_INTERVAL != 0) && (HAL_GetTick() % rad_params.HEALTH_INTERVAL == 0))
        {
            rad_status.RAD_STATE = rad_state;
            rad_status.TMC_STATUS = TMC_2590_CheckState(&tmc_2590_1);
            MX_CAN_Broadcast_Health_Message(&rad_can, rad_status);
        }

        rad_can.timer += HAL_GetTick() - prev_ms;
        prev_ms = HAL_GetTick();

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    TMC_2590_TIM_PWM_PulseFinishedCallback(&tmc_2590_1, htim);
}

void MOTION_PROFILE_Set_Speed(uint16_t velocity) {

    // Guard against divide-by-zero if velocity drops to 0
    if (velocity == 0) {
        return;
    }

	// Calculate ARR from inputted desired freq
	uint32_t arr = HAL_TIM_CalculateAutoReload(tmc_2590_1.Init.STEP_Tim, velocity);

	// Assign ARR to timer hardware
	TMC_2590_SetTimAutoReload(&tmc_2590_1, arr);

	// Update local stepper speed reference
	rad_params.STEPPER_SPEED = HAL_TIM_CalculateFrequency(tmc_2590_1.Init.STEP_Tim);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
