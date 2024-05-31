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
#include <string.h>
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
int speed;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void Callback_CAN(CAN_MESSAGE *msg);
void Callback_LED(CAN_MESSAGE *msg);
void Callback_Stepper(CAN_MESSAGE *msg);

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
//  MX_GPIO_Init();
//  MX_CAN_Init();
//  MX_I2C1_Init();
//  MX_SPI1_Init();
//  MX_SPI2_Init();
//  MX_TIM2_Init();
//  MX_ADC1_Init();
//  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    static enum
    {
      BAREBONES_STATE_CAN = 0,
      BAREBONES_STATE_LED,
      BAREBONES_STATE_STEPPER_CAN
    } eState = BAREBONES_STATE_STEPPER_CAN; //Adjust test state here


    switch (eState)
    {
      case BAREBONES_STATE_CAN:
      {

        CAN_Initialize();
        CAN_RegisterReceiveCallback(&Callback_CAN);

        break;
      }
      case BAREBONES_STATE_LED:
      {

        CAN_Initialize();
        CAN_RegisterReceiveCallback(&Callback_LED);
        break;
      }

      case BAREBONES_STATE_STEPPER_CAN:
      {

        CAN_Initialize();
        STEPPER_Initialize();
        STEPPER_AdjustStepSpeed(200); //200 hz

        STEPPER_REGISTER_DATA data = {0};

        //all values are zero - only need to populate non-zero values

        data.reg.drvconf.SLP = 0b11110;
        data.reg.drvconf.RDSEL = 0b11;
        data.reg.drvconf.SHRTSENS = 1;
        data.reg.drvconf.EN_PFD = 1;
        data.reg.drvconf.EN_S2VS = 1;

        data.reg.sgconf.SGT = 0b0000010;
        data.reg.sgconf.CS = 7;

        //smarten is all 0

        data.reg.chopconf.TBL = 0b10;
        data.reg.chopconf.HEND = 0b0100;
        data.reg.chopconf.HSTRT = 0b110;
        data.reg.chopconf.TOFF = 0b0100;

        data.reg.drvctrl.INTPOL = 1;
        data.reg.drvctrl.MRES = 0b0111;




        STEPPER_WriteRegisterConfig(STEPPER_REGISTER_ALL, &data);
        CAN_RegisterReceiveCallback(&Callback_Stepper);
        break;
      }
    }

    uint32_t timer = HAL_GetTick();
    STEPPER_DIRECTION dir = STEPPER_DIRECTION_CW;

    while (1)
    {



    }

    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Callback_CAN(CAN_MESSAGE *msg)
{
  CAN_MESSAGE txMsg;
  memset(&txMsg.data, 0, sizeof(txMsg.data));

  HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);

  txMsg.header = 3;
  //data is zero

  CAN_Send(&txMsg);

}

void Callback_LED(CAN_MESSAGE *msg)
{

  switch(msg->header)
  {
    case 0x54:
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
      break;
    case 0x55:
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
      break;
    default:
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
      break;
  }

  CAN_MESSAGE txMsg;

  txMsg.header = 0x03;
  memcpy(txMsg.data, msg->data, 8); //echo data w a different header

  CAN_Send(&txMsg);
}

void Callback_Stepper(CAN_MESSAGE *msg)
{


	uint16_t hz = (msg->data[6] << 8) | msg->data[7];
  switch(msg->header)
  {
    case 0x54:
    {
    	if (hz != 0)
		{
    		STEPPER_SetDirection(STEPPER_DIRECTION_CCW);
			STEPPER_AdjustStepSpeed(hz);
			STEPPER_StartStep();
		}
		else
		{
			STEPPER_StopStep();
		}

      break;
    }
    case 0x55:
      //CW Direction
	{
		if (hz != 0)
		{
			STEPPER_SetDirection(STEPPER_DIRECTION_CW);
			STEPPER_AdjustStepSpeed(hz);
			STEPPER_StartStep();
		}
		else
		{
			STEPPER_StopStep();
		}

      break;
	}
    default:
    	STEPPER_StopStep();
      break;
  }

  CAN_MESSAGE txMsg;

  txMsg.header = 0x03;
  memcpy(txMsg.data, msg->data, 8); //echo data w a different header

  CAN_Send(&txMsg);
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
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);

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
