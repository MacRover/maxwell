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
#include "can.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "at24c04c.h"
#include "queue.h"
#include "enc_dec_utils.h"

// todo: add more if needed

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VIPER_PARAMS_EEPROM_PAGE 0


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

VIPER_STATUS_TypeDef viper_status;
VIPER_PARAMS_TypeDef viper_params;

uint8_t ESTOP = 0;
uint8_t DISABLED = 0;

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

	// ##############

	// todo: SET DEFAULT VIPER PARAMS HERE

    viper_params.HEALTH_INTERVAL = 1000; //every second
    viper_params.CARD_INTERVAL = 20; // may need to tune this


	// ###############

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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


  	// #############

  	// SET INITIAL EEPROM READ HERE (?)


	// ##############

	// SET DRIVER INITIALIZATIONS HERE

	// ###############

  	// 	BROADCAST HEALTH MESSAGE HERE

	// ###############


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  if (!queue_empty(&can_message_queue_global))
	  {
		  VIPER_CAN_Message_TypeDef *new_message =
				  (VIPER_CAN_Message_TypeDef*) queue_front(
						  &can_message_queue_global);

		  switch ((int)(new_message->command_id))
		  {
			  case ESTOP_MESSAGE:
			  {
				  ESTOP = 1;
				  break;
			  }
			  case DISABLE_MESSAGE:
			  {
				  DISABLED = 1;
				  break;
			  }
			  case ENABLE_MESSAGE:
			  {
				  viper_state = VIPER_STATE_ACTIVE;
				  DISABLED = 0;
				  break;
			  }
			  case HEALTH_STATUS_PING:
			  {
				  MX_CAN_Broadcast_Health_Message(&viper_can, viper_status);
				  break;
			  }
			  default:
			  {
				  break;
			  }
		  }

		  free(new_message->data);
		  queue_dequeue(&can_message_queue_global);
	  }

	  //Only check viper queue after. This allows global messages to be addressed immediately
	  else if (!queue_empty(&can_message_queue_viper))
	  {
		  VIPER_CAN_Message_TypeDef *new_message =
				  (VIPER_CAN_Message_TypeDef*) queue_front(
						  &can_message_queue_viper);

		  switch ((int)(new_message->command_id))
		  {
		  	  case GET_CARD_1_STATUS:
		  	  {
		  		break;
		  	  }
		  	  case GET_CARD_2_STATUS:
		  	  {
		  		break;
		  	  }
		  	  case GET_CARD_3_STATUS:
		  	  {
		  		break;
		  	  }
		  	  case GET_CARD_4_STATUS:
		  	  {
		  		break;
		  	  }
		  	  case SYNC_EEPROM:
		  	  {
		  		break;
		  	  }
		  	  case CLEAR_EEPROM:
		  	  {
		  		break;
		  	  }
		  	  case GET_EEPROM_STATUS:
		  	  {
		  		break;
		  	  }
		  	  case GET_HEALTH_STATUS:
		  	  {
		  		break;
		  	  }
		  	  case GET_MUX_STATUS:
		  	  {
		  		break;
		  	  }
		  	  case GET_HEALTH_INTERVAL:
		  	  {
		  		break;
		  	  }
		  	  case SET_HEALTH_INTERVAL:
		  	  {
		  		break;
		  	  }
		  	  case GET_CARD_INTERVAL:
		  	  {
		  		break;
		  	  }
		  	  case SET_CARD_INTERVAL:
		  	  {
		  		break;
		  	  }
		  	  case GET_VIPER_ID:
		  	  {
		  		break;
		  	  }
		  	  case SET_VIPER_ID:
		  	  {
		  		break;
		  	  }
		  	  default:
		  		  break;
		  }
		  free(new_message->data);
		  queue_dequeue(&can_message_queue_viper);
	  }
	  //CHECK FOR ESTOP
	  // todo
//	  if (ESTOP)
//	  {
//		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
//		  break;
//	  }
//	  if (DISABLED)
//	  {
//		steps_to_move = 0;
//		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
//		  continue;
//	  }
//	  else
//	  {
//		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
//	  }

	switch (viper_state) {
		case VIPER_STATE_INIT: {
			break;
		}
		case VIPER_STATE_ACTIVE: {
			break;
		}
		default:
			break;
	}

	if ((viper_params.CARD_INTERVAL != 0) && (HAL_GetTick() % viper_params.CARD_INTERVAL == 0))
	{
		//todo replace with card status function
		//MX_CAN_Broadcast_Odometry_Message(&viper_can, viper_status);
	}

	if ((viper_params.HEALTH_INTERVAL != 0) && (HAL_GetTick() % viper_params.HEALTH_INTERVAL == 0))
	{
		viper_status.VIPER_STATE = viper_state;
		MX_CAN_Broadcast_Health_Message(&viper_can, viper_status);
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // LED STATUS BLINK ----------------------------------------------------

	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	  HAL_Delay(1000);

	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	  HAL_Delay(1000);

	  // LED STATUS BLINK ----------------------------------------------------





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
}

/* USER CODE BEGIN 4 */

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