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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//#define LS_INVERT_DIR
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;

// CAN Rx
CAN_FilterTypeDef canfilterconfig;
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
int speed;
uint32_t timer;
LS_STATE ls_state = RELEASED;

uint8_t can_id = 0x2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
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
    canfilterconfig.FilterMaskIdLow = (0x07FE) << 3;
    canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    canfilterconfig.FilterActivation = ENABLE;
    canfilterconfig.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); //enable interrupts
}

void txCAN()
{
    HAL_StatusTypeDef can_result = HAL_CAN_AddTxMessage(&hcan, &TxHeader,
            TxData, &TxMailbox);
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

    timer = 0;

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
  MX_SPI1_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // Motor driver chip enable
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // disable stand alone mode
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // CS

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); // set DIR
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); // STEP

    uint32_t SPImsg = 0;
    uint8_t SPImsg_bytes[3];

//  uint32_t SPIread;
    uint8_t SPIread_bytes[3];

    HAL_StatusTypeDef spi_status;

    // Bits 19, 18, 17
    // 111 DRVCONF (0b111 << 17)
    // 110 SGCSCONF (0b110 << 17)
    // 101 SMARTEN (0b101 << 17)
    // 100 CHOPCONF (0b100 << 17)
    // 00X DRVCTRL (0b00 << 18)

    /* DRVCONF */
    SPImsg = 0;
    SPImsg |= (0b111 << 17); // DRVCONF
    SPImsg |= (0b0 << 16); // TST: test mode
    SPImsg |= (0b11110 << 11); // SLP: Slope control
    SPImsg |= (0b0 << 10); // DIS_S2G: Short to ground protection
    SPImsg |= (0b00 << 8); // TS2G: Short detection delay
    SPImsg |= (0b0 << 7); // SDOFF: step/dir interface
    SPImsg |= (0b0 << 6); // VSENSE: full-scale sense resistor voltage setting
    SPImsg |= (0b11 << 4); // RDSEL: read out select
    SPImsg |= (0b0 << 3); // OTSENS: overtemp shutdown setting
    SPImsg |= (0b1 << 2); // SHRTSENS: short to ground sensitivity
    SPImsg |= (0b1 << 1); // EN_PFD: passive fast delay setting
    SPImsg |= (0b1 << 0); // EN_S2VS: Short to VS protection

    SPImsg_bytes[2] = (uint8_t) (SPImsg & 0xFF);
    SPImsg_bytes[1] = (uint8_t) ((SPImsg & 0xFF00) >> 8);
    SPImsg_bytes[0] = (uint8_t) ((SPImsg & 0xFF0000) >> 16);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    spi_status = HAL_SPI_TransmitReceive(&hspi1, SPImsg_bytes, SPIread_bytes, 3,
            1000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    /* SGCSCONF */
    SPImsg = 0;
    SPImsg |= (0b110 << 17); // SGCSCONF
    SPImsg |= (0b0 << 16); // SFILT: stall guard filter
    SPImsg |= (0b0000010 << 8); // SGT: stall guard threshold
    SPImsg |= (10 << 0); // CS: current scale

    SPImsg_bytes[2] = (uint8_t) (SPImsg & 0xFF);
    SPImsg_bytes[1] = (uint8_t) ((SPImsg & 0xFF00) >> 8);
    SPImsg_bytes[0] = (uint8_t) ((SPImsg & 0xFF0000) >> 16);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    spi_status = HAL_SPI_TransmitReceive(&hspi1, SPImsg_bytes, SPIread_bytes, 3,
            1000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    /* SMARTEN */
    SPImsg = 0;
    SPImsg |= (0b101 << 17); // SMARTEN
    SPImsg |= (0b0 << 15); // SEIMIN: min cool step current
    SPImsg |= (0b00 << 13); // SEDN: current dec. speed
    SPImsg |= (0b0000 << 8); // SEMAX: upper cool step threshold offset
    SPImsg |= (0b00 << 5); // SEUP: current increment size
    SPImsg |= (0b0000 << 0); // SEMIN: cool step lower threshold

    SPImsg_bytes[2] = (uint8_t) (SPImsg & 0xFF);
    SPImsg_bytes[1] = (uint8_t) ((SPImsg & 0xFF00) >> 8);
    SPImsg_bytes[0] = (uint8_t) ((SPImsg & 0xFF0000) >> 16);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    spi_status = HAL_SPI_TransmitReceive(&hspi1, SPImsg_bytes, SPIread_bytes, 3,
            1000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    /* CHOPCONF */
    SPImsg = 0;
    SPImsg |= (0b100 << 17); // CHOPCONF
    SPImsg |= (0b10 << 15); // TBL: blanking time
    SPImsg |= (0b0 << 14); // CHM: chopper mode
    SPImsg |= (0b0 << 13); // RNDTF: Random TOFF time
    SPImsg |= (0b00 << 11); // HDEC: hysteresis decay or fast decay mode
    SPImsg |= (0b0100 << 7); // HEND: hysteresis end value
    SPImsg |= (0b110 << 4); // HSTRT: hysteresis start value
    SPImsg |= (0b0100 << 0); // TOFF: mosfet off time

    SPImsg_bytes[2] = (uint8_t) (SPImsg & 0xFF);
    SPImsg_bytes[1] = (uint8_t) ((SPImsg & 0xFF00) >> 8);
    SPImsg_bytes[0] = (uint8_t) ((SPImsg & 0xFF0000) >> 16);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    spi_status = HAL_SPI_TransmitReceive(&hspi1, SPImsg_bytes, SPIread_bytes, 3,
            1000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    /* DRVCTRL */
    SPImsg = 0;
    SPImsg |= (0b00 << 18); // DRVCTRL
    SPImsg |= (0b1 << 9); // INTPOL: step interpolation
    SPImsg |= (0b0 << 8); // DEDGE: Double edge step pulses
    SPImsg |= (0b0111 << 0); // MRES: Microsteps per fullstep

    SPImsg_bytes[2] = (uint8_t) (SPImsg & 0xFF);
    SPImsg_bytes[1] = (uint8_t) ((SPImsg & 0xFF00) >> 8);
    SPImsg_bytes[0] = (uint8_t) ((SPImsg & 0xFF0000) >> 16);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    spi_status = HAL_SPI_TransmitReceive(&hspi1, SPImsg_bytes, SPIread_bytes, 3,
            1000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    HAL_Delay(2000);

    setupTxCAN();
    setupRxCAN();

    uint32_t prev = 0;
    uint32_t delay;

    //LOCK THE MOTOR WITH THIS SETTING:
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // Motor driver chip enable


	while (1) {
		timer += HAL_GetTick() - prev;
		prev = HAL_GetTick();

		if (speed == 0 || timer > 500)
		{
		    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); // Motor driver chip disable so we can free spin it

//			txCAN();

			HAL_Delay(10);
//			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
		}
		else if (speed > 0)
		{
#ifndef LS_INVERT_DIR
			if (ls_state == PRESSED) continue;
#endif
		    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // Motor driver chip enable

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

			delay = 100000/speed; //arbitrary speed calculation

			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3); // STEP
//			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2); //LED


			delayUs(delay);

		}
		else if (speed < 0)
		{
#ifdef LS_INVERT_DIR
			if (ls_state == PRESSED) continue;
#endif
		    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // Motor driver chip enable

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

			delay = 100000/ (-1*speed);

			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3); // STEP
//			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2); //LED

			delayUs(delay);
		}



	//HAL_Delay(1);


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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LS_1_Pin|DRIVER_CS_Pin|DRIVER_ENN_Pin|DRIVER_ST_ALONE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_RED_Pin|DRIVER_STEP_Pin|DRIVER_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LS_1_Pin DRIVER_CS_Pin DRIVER_ENN_Pin DRIVER_ST_ALONE_Pin */
  GPIO_InitStruct.Pin = LS_1_Pin|DRIVER_CS_Pin|DRIVER_ENN_Pin|DRIVER_ST_ALONE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LS_2_Pin */
  GPIO_InitStruct.Pin = LS_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LS_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED_Pin DRIVER_STEP_Pin DRIVER_DIR_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|DRIVER_STEP_Pin|DRIVER_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DRIVER_SG_TEST_Pin */
  GPIO_InitStruct.Pin = DRIVER_SG_TEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DRIVER_SG_TEST_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == LS_2_Pin)
	{
		if (HAL_GPIO_ReadPin(LS_2_GPIO_Port, GPIO_Pin) == GPIO_PIN_SET)
		{
			ls_state = PRESSED;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
		}
		else if (HAL_GPIO_ReadPin(LS_2_GPIO_Port, GPIO_Pin) == GPIO_PIN_RESET)
		{
			ls_state = RELEASED;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
		}
	}
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

    uint32_t CAN_ERR = HAL_CAN_GetError(&hcan);

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
