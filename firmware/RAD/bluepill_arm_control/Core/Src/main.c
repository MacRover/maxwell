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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


//ENUM MUST MATCH DESIRED JOINT IDs
//ORDER IS NOT ABRITRARY
typedef enum
{
	RAD_JOINT_WRIST_FLEX = 0,
	RAD_JOINT_GRIPPER,
	RAD_JOINT_BASE,
	RAD_JOINT_ELBOW,
	RAD_JOINT_WRIST_ROTATE,
	RAD_JOINT_SHOULDER
} RAD_JOINT_NUMBER;

typedef struct
{
	RAD_JOINT_NUMBER id;
	GPIO_TypeDef *step_port;
	uint16_t step_pin;
	GPIO_TypeDef *dir_port;
	uint16_t dir_pin;
	uint16_t isHigh;

} RAD_JOINT;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */

CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;

// CAN Rx
CAN_FilterTypeDef canfilterconfig;
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
int speed;
uint32_t can_timer;
uint32_t cycle_timer;

uint32_t half_cycle_time_us = 1000;

uint8_t can_id = 0x18;

RAD_JOINT_NUMBER mostRecentJointId;
RAD_JOINT *mostRecentJoint;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void setupTxCAN()
{
    TxHeader.StdId = 0x321;
    TxHeader.ExtId = can_id;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;

    TxData[0] = 0xA0;
    TxData[1] = 0xA1;
    TxData[2] = 0xA2;
    TxData[3] = 0xA3;
    TxData[4] = 0xA4;
    TxData[5] = 0xA5;
    TxData[6] = 0xA6;
    TxData[7] = 0xA9;
}

void setupRxCAN()
{
    canfilterconfig.FilterBank = 0;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilterconfig.FilterIdHigh = 0x0000;
    canfilterconfig.FilterIdLow = ((can_id) << 8 | 0x0054) << 3;
    canfilterconfig.FilterMaskIdHigh = 0x0000;
    canfilterconfig.FilterMaskIdLow = (0x78FE) << 3;
    canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    canfilterconfig.FilterActivation = ENABLE;
    canfilterconfig.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); //enable interrupts
}

void txCAN()
{
    HAL_StatusTypeDef can_result = HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);

    if (can_result != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
    /* Get RX message */
    if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &RxHeader, RxData)
            != HAL_OK)
    {
        /* Reception Error */
        Error_Handler();
    }

    can_timer = 0;

//    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
    // Echo received CAN message

    //RxData[7] = RxData[7] + 1;
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, RxData, &TxMailbox);

    if ((RxHeader.ExtId & 0xFF) == 0x54)
    {
    	//counter clockwise

    	if (RxData[7] != 0)
    	{
    		speed = 0 + RxData[7];
    	}
    	else
    	{
    		speed = 0;
    	}

    	mostRecentJointId = (RxHeader.ExtId & 0x700) >> 8;
    }
    else if ((RxHeader.ExtId & 0xFF) == 0x55)
    {
    	//clockwise clockwise

    	if (RxData[7] != 0)
    	{
    		speed = 0 - RxData[7];
    	}
    	else
    	{
    		speed = 0;
    	}

    	mostRecentJointId = (RxHeader.ExtId & 0x700) >> 8;
    }

}



uint32_t getUs(void)
{

    uint32_t usTicks = HAL_RCC_GetSysClockFreq() / 1000000;

    register uint32_t ms, cycle_cnt;
    do
    {
        ms = HAL_GetTick();
        cycle_cnt = SysTick->VAL;
    } while (ms != HAL_GetTick());
    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

void delayUs(uint16_t micros)
{
    uint32_t start = getUs();
    while (getUs() - start < (uint32_t) micros)
    {
        asm("nop");
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	RAD_JOINT base = (RAD_JOINT){RAD_JOINT_BASE, BASE_STEP_GPIO_Port, BASE_STEP_Pin, BASE_DIR_GPIO_Port, BASE_DIR_Pin};
	RAD_JOINT elbow  = (RAD_JOINT){RAD_JOINT_ELBOW, ELBOW_STEP_GPIO_Port, ELBOW_STEP_Pin, ELBOW_DIR_GPIO_Port, ELBOW_DIR_Pin, 0};
	RAD_JOINT wrist  = (RAD_JOINT){RAD_JOINT_WRIST_FLEX, WRIST_STEP_GPIO_Port, WRIST_STEP_Pin, WRIST_DIR_GPIO_Port, WRIST_DIR_Pin};
	RAD_JOINT gripper  = (RAD_JOINT){RAD_JOINT_GRIPPER, GRIPPER_STEP_GPIO_Port, GRIPPER_STEP_Pin, GRIPPER_DIR_GPIO_Port, GRIPPER_DIR_Pin};


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
  /* USER CODE BEGIN 2 */

  setupTxCAN();
  setupRxCAN();

	uint32_t prev = 0;
    uint32_t delay;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  can_timer += HAL_GetTick() - prev;
	  cycle_timer += HAL_GetTick() - prev;
	  prev = HAL_GetTick();

	  if (can_timer % 1000)
	  {
		  TxData[5] = mostRecentJoint->id;
		  TxData[6] = speed;
		  TxData[7] = mostRecentJoint->isHigh;
		  txCAN();
		  HAL_GPIO_TogglePin(LED_PIN_GPIO_Port, LED_PIN_Pin);

	  }


	  if (speed != 0 || can_timer > 4000)
	  {
		  //CAN timeout
		  speed = 0;
	  }


	  //mostRecentJointId = 3;
	  switch (mostRecentJointId)
	  {
		  case RAD_JOINT_BASE:
		  {
			  mostRecentJoint = &base;
			  break;
		  }
		  case RAD_JOINT_WRIST_FLEX:
		  {
			  mostRecentJoint = &wrist;
			  break;
		  }
		  case RAD_JOINT_ELBOW:
		  {
			  mostRecentJoint = &elbow;
			  break;
		  }
		  case RAD_JOINT_GRIPPER:
		  {
			  mostRecentJoint = &gripper;
			  break;
		  }
		  default:
			  speed = 0;
			  break;

	  }

	  //speed = 0;

//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
//
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
//
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
//
//	  HAL_Delay(1000);
//
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
//	  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
//
//	  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
//	  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
//
//	  	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
//	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
//	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
//	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
//	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
//	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

	  	//HAL_Delay(1000);

	delay = 3.9*(255 - speed) + 3; //arbitrary speed calculation


	  if (speed > 0)
	  {

		  HAL_GPIO_WritePin(mostRecentJoint->dir_port, mostRecentJoint->dir_pin, GPIO_PIN_SET);

		  if (cycle_timer > delay && mostRecentJoint->isHigh)
		  {
			  HAL_GPIO_WritePin(mostRecentJoint->step_port, mostRecentJoint->step_pin, GPIO_PIN_RESET);
			  mostRecentJoint->isHigh = 0;
			  cycle_timer = 0;
		  }
		  else if (cycle_timer > delay && !(mostRecentJoint->isHigh))
		  {
			  HAL_GPIO_WritePin(mostRecentJoint->step_port, mostRecentJoint->step_pin, GPIO_PIN_SET);
			  mostRecentJoint->isHigh = 1;
			  cycle_timer = 0;
		  }
	  }
	  else if (speed < 0)
	  {

		  HAL_GPIO_WritePin(mostRecentJoint->dir_port, mostRecentJoint->dir_pin, GPIO_PIN_RESET);

		  if (cycle_timer > delay && mostRecentJoint->isHigh)
		  {
			  HAL_GPIO_WritePin(mostRecentJoint->step_port, mostRecentJoint->step_pin, GPIO_PIN_RESET);
			  mostRecentJoint->isHigh = 0;
			  cycle_timer = 0;
		  }
		  else if (cycle_timer > delay && !(mostRecentJoint->isHigh))
		  {
			  HAL_GPIO_WritePin(mostRecentJoint->step_port, mostRecentJoint->step_pin, GPIO_PIN_SET);
			  mostRecentJoint->isHigh = 1;
			  cycle_timer = 0;
		  }
	  }

//
//

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  if (HAL_CAN_Start(&hcan) != HAL_OK)
{
	/* Start Error */
	Error_Handler();
}

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
  //__HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_AFIO_REMAP_SWJ_NOJTAG();
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|WRIST_STEP_Pin|WRIST_DIR_Pin
                          |BASE_STEP_Pin|BASE_DIR_Pin|GPIO_PIN_6|GPIO_PIN_7
                          |ELBOW_STEP_Pin|ELBOW_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GRIPPER_STEP_Pin|GRIPPER_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_PIN_Pin */
  GPIO_InitStruct.Pin = LED_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 WRIST_STEP_Pin WRIST_DIR_Pin
                           BASE_STEP_Pin BASE_DIR_Pin PB6 PB7
                           ELBOW_STEP_Pin ELBOW_DIR_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|WRIST_STEP_Pin|WRIST_DIR_Pin
                          |BASE_STEP_Pin|BASE_DIR_Pin|GPIO_PIN_6|GPIO_PIN_7
                          |ELBOW_STEP_Pin|ELBOW_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GRIPPER_STEP_Pin GRIPPER_DIR_Pin */
  GPIO_InitStruct.Pin = GRIPPER_STEP_Pin|GRIPPER_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
