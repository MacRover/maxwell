/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "can.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "motion.h"
#include "tmc_2590.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

void LS_Released_Callback(LS_NUMBER *num);
void LS_Pressed_Callback(LS_NUMBER *num);


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

RAD_PARAMS_TypeDef rad_params;
RAD_MOTION_PROFILE_TypeDef rad_motion_profile;

uint8_t ESTOP = 0;
uint8_t DISABLED = 0;

uint16_t min_angle;
uint16_t max_angle;
uint16_t gearing;
uint16_t max_rotations;
uint16_t steps_per_revolution;

float software_stop = 0;
uint8_t cw_enable = 0;
uint8_t ccw_enable = 0;

double* angle_average_buffer;
uint8_t buffer_head;

int16_t steps_to_move;


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

	  //SET DEFAULT VALUES

		rad_params.RAD_ID = 0xF0;
	    rad_params.RAD_TYPE = RAD_TYPE_UNDEFINED;
	    rad_params.HOME_POSITION = RAD_TYPE_DRIVETRAIN_MAX_ROTATIONS/2;
	    rad_params.STEPPER_SPEED = 1000;
	    rad_params.ODOM_INTERVAL = 20; //50hz, or 20ms
	    rad_params.HEALTH_INTERVAL = 1000; //every second
	    rad_params.P = 0.01;
	    rad_params.I = 0.0000001;
	    rad_params.D = 0;

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

	    rad_params.SGCSCONF_CS = 10;
	    rad_params.SGCSCONF_SFILT = 0b0;
	    rad_params.SGCSCONF_SGT = 0b0000010;

	    rad_params.SMARTEN_SEDN = 0b00;
	    rad_params.SMARTEN_SEIMIN = 0b0;
	    rad_params.SMARTEN_SEMAX = 0b0000;
	    rad_params.SMARTEN_SEMIN = 0b0000;
	    rad_params.SMARTEN_SEUP = 0b00;

	    rad_params.PID_MIN_OUTPUT = 20;
	    rad_params.PID_MAX_OUTPUT = 1000;

	    rad_params.HOME_OFFSET = 0;

	    rad_motion_profile.STEPS_TO_MOVE = 90;
	    rad_motion_profile.V_I = 0;
	    rad_motion_profile.V_MAX = 5;
	    rad_motion_profile.ACCELERATION = 2;
	    rad_motion_profile.CURRENT_POS = 0;
	    rad_motion_profile.TIME_ELAPSED = 0;
	    rad_motion_profile.VELOCITY = 0;
	    rad_motion_profile.MOVEMENT_STEPS = 0;
	    rad_motion_profile.TOTAL_STEPS = 0;

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
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

    // Motion_Profiler_HandleTypeDef profile;
    
    // profile.Init.max_velocity = 10;
    // profile.Init.min_velocity = 0;
    
    // profile.Init.acceleration = 5;
    // profile.Init.deadzone = 1;
    
    // Motion_Profiler_Init(&profile);
    
    
    
    // double start = 0;
    // double end = 10;
    
    // Motion_Profiler_GenerateNewTrajectory(&profile, start, end);
    
    // printf("Start: %f\n", profile.start);
    // printf("Setpoint: %f\n", profile.setpoint);

    // printf("blend x start: %f\n", profile.blend_x_start);

    // printf("blend x end: %f\n", profile.blend_x_end);

    
    // for (int i = 0; i <= end; i ++)
    // {
    //     Motion_Profiler_GetVelocity(&profile, (double)i);
    //     printf("Position: %d, Velocity: %d, Current State: %d\n", i, profile.current_velocity, profile.State);
    // }
    

    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
      
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    	HAL_TIM_Base_Start(&htim2);


    	// On first run, v_i = 0

    	while (rad_motion_profile.TOTAL_STEPS < rad_motion_profile.STEPS_TO_MOVE) {


			rad_motion_profile.TIME_ELAPSED = __HAL_TIM_GET_COUNTER(&htim2);

			rad_motion_profile.VELOCITY = MOTION_PROFILE_VELOCITY(rad_motion_profile.CURRENT_POS, rad_motion_profile.STEPS_TO_MOVE, rad_motion_profile.ACCELERATION, rad_motion_profile.V_I, rad_motion_profile.V_MAX, rad_motion_profile.TIME_ELAPSED);

			rad_motion_profile.MOVEMENT_STEPS = rad_motion_profile.VELOCITY * rad_motion_profile.TIME_ELAPSED;

			rad_motion_profile.TOTAL_STEPS += rad_motion_profile.MOVEMENT_STEPS;

			// Moving steps go here.  Need to link

			// todo (in hatch) - figure out the linking

			rad_motion_profile.V_I = rad_motion_profile.VELOCITY;
    	}










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

void LS_Pressed_Callback(LS_NUMBER *num)
{
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
}

void LS_Released_Callback(LS_NUMBER *num)
{
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
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
