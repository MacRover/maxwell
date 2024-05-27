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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
// CAN Tx
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;

// CAN Rx
CAN_FilterTypeDef canfilterconfig;
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void setupTxCAN()
{
  TxHeader.StdId = 0x321;
  TxHeader.ExtId = 0x01;
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
  TxData[7] = 0xA7;
}

void setupRxCAN()
{
  canfilterconfig.FilterBank = 0;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.FilterIdHigh = 0x0000;
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x0000;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterActivation = ENABLE;
  canfilterconfig.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // enable interrupts
}

void txCAN()
{
  TxHeader.DLC = 8;
  HAL_StatusTypeDef can_result = HAL_CAN_AddTxMessage(&hcan, &TxHeader,
                                                      TxData, &TxMailbox);
  if (can_result != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief  Rx Fifo 0 message pending callback in non blocking mode
 * @param  CanHandle: pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
  uint8_t echo_data[8];
  /* Get RX message */
  if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }

  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
  // Echo received CAN message
  HAL_CAN_AddTxMessage(&hcan, &TxHeader, echo_data, &TxMailbox);
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
  MX_CAN_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  setupTxCAN();
  setupRxCAN();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    //txCAN();
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);

    // set channel
    uint8_t device_identifier = 0b11100000;
    uint8_t a2_pin = 0;
    uint8_t a1_pin = 0;
    uint8_t a0_pin = 0;
    uint8_t read_write = 0; // write

    uint8_t i2c_address = device_identifier | (a2_pin << 3) | (a1_pin << 2) | (a0_pin << 1) | read_write;

    uint8_t channel = 1;

    uint8_t i2c_send_data_mux = 0b00000000 | (channel | 4);

    while (HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)i2c_address, &i2c_send_data_mux, 1, 10000) != HAL_OK)
    {
      if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_AF)
      {
        Error_Handler();
      }
    }

    //shift i/o mux
    //configure input/output
    device_identifier = 0b00110000;
    a2_pin = 0;
    a1_pin = 0;
    a0_pin = 0;
    read_write = 0; // write

    i2c_address = device_identifier | (a2_pin << 3) | (a1_pin << 2) | (a0_pin << 1) | read_write;

    uint8_t i2c_send_data_io[2];
    i2c_send_data_io[0] = (uint8_t )0b00000011; //configuration register
    i2c_send_data_io[1] = (uint8_t) 0b00011111; // 7-5 output, 1-4 input, 0 NC

    while (HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)i2c_address, &i2c_send_data_io[0], 2, 10000) != HAL_OK)
    {
      if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_AF)
      {
        Error_Handler();
      }
    }


    //configure input polarity

    i2c_send_data_io[0] = (uint8_t) 0b00000010; //output port register
    i2c_send_data_io[1] = (uint8_t) 0b00000000; // none are inverted

    while (HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)i2c_address, &i2c_send_data_io[0], 2, 10000) != HAL_OK)
    {
      if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_AF)
      {
        Error_Handler();
      }
    }

    //configure output channels
    i2c_send_data_io[0] = (uint8_t) 0b00000001; //output port register
    i2c_send_data_io[1] = (uint8_t) 0b11100000; // 7-5 on

    while (HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)i2c_address, &i2c_send_data_io[0], 2, 10000) != HAL_OK)
    {
      if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_AF)
      {
        Error_Handler();
      }
    }

    //configure register for reading
    i2c_send_data_io[0] = (uint8_t) 0b00000000; //input port register
    //i2c_send_data_io[1] = (uint8_t) 0b11100000; // Doesn't respond to second byte - only takes one

    while (HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)i2c_address, &i2c_send_data_io[0], 1, 10000) != HAL_OK)
    {
      if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_AF)
      {
        Error_Handler();
      }
    }
    //NO DELAY BETWEEN THESE COMMANDS

	//read io mux
	device_identifier = 0b00110000;
	a2_pin = 0;
	a1_pin = 0;
	a0_pin = 0;
	read_write = 1; // read

	i2c_address = device_identifier | (a2_pin << 3) | (a1_pin << 2) | (a0_pin << 1) | read_write;

	uint8_t i2c_read_data_io[1];

	while (HAL_I2C_Master_Receive(&hi2c2, (uint16_t)(i2c_address), i2c_read_data_io, 1, 10000) != HAL_OK)
	{
	  if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_AF)
	  {
		Error_Handler();
	  }
	}




     write eeprom
    uint8_t read_eeprom = 0; // address in eeprom to write to
    uint16_t eeprom_address = 321;

    device_identifier = 0b10100000;
    a2_pin = 0;
    a1_pin = 0;

    i2c_address = device_identifier | (a2_pin << 3) | (a1_pin << 2) | (eeprom_address >> 8) << 1 | read_eeprom;

    uint8_t i2c_send_data[2];
    i2c_send_data[0] = (uint8_t)eeprom_address & 0xff;
    i2c_send_data[1] = (uint8_t)channel;

    while (HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)i2c_address,
                                   i2c_send_data, sizeof(i2c_send_data), 10000) != HAL_OK)
    {
      /* Error_Handler() function is called when Timeout error occurs.
       When Acknowledge failure occurs (Slave don't acknowledge its address)
       Master restarts communication */
      if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_AF)
      {
        Error_Handler();
      }
    }

    

    HAL_Delay(25);

    eeprom_address = 321; // address in eeprom to read from
    i2c_address = device_identifier | (a2_pin << 3) | (a1_pin << 2) | (eeprom_address >> 8) << 1 | read_eeprom;
    uint8_t i2c_address_write = (uint8_t)(i2c_address & 0xfe);

    uint8_t i2c_data_buff;
    while (HAL_I2C_Mem_Read(&hi2c2, (uint16_t)i2c_address_write,
                            (uint16_t)eeprom_address, I2C_MEMADD_SIZE_8BIT, &i2c_data_buff,
                            1, 10000) != HAL_OK)
    {
      /* Error_Handler() function is called when Timeout error occurs.
       When Acknowledge failure occurs (Slave don't acknowledge its address)
       Master restarts communication */
      if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_AF)
      {
        Error_Handler();
      }
    }



    uint8_t can_data[1];
    can_data[0] = i2c_data_buff;
    TxHeader.DLC = 1;
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, can_data, &TxMailbox);


    can_data[0] = i2c_read_data_io;
    TxHeader.DLC = 1;
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, can_data, &TxMailbox);
    HAL_Delay(500);
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
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  __HAL_AFIO_REMAP_SWJ_NOJTAG();
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

	 //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
	  uint32_t HAL_ERR = HAL_I2C_GetError(&hi2c2);

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
