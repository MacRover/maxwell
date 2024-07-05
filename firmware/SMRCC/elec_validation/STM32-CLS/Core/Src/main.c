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
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COMMAND_VEML_RED 0x05
#define COMMAND_VEML_GREEN 0x06
#define COMMAND_VEML_BLUE 0x07

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t command;
uint8_t COMMAND_CODE_RED = 0x05;
uint8_t buf[20];
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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // set channel
	uint8_t device_identifier = 0b11100000;
	uint8_t a2_pin = 0;
	uint8_t a1_pin = 0;
	uint8_t a0_pin = 0;
	uint8_t read_write = 0; // write

	uint8_t i2c_address = device_identifier | (a2_pin << 3) | (a1_pin << 2) | (a0_pin << 1) | read_write;

	uint8_t channel = 2;

	uint8_t i2c_send_data_mux = 0b00000000 | (channel | 4);

	while (HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)i2c_address, &i2c_send_data_mux, 1, 10000) != HAL_OK)
	{
		if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_AF)
		{
			Error_Handler();
		}
	}
	uint8_t pca = 0b00110000;
	uint8_t msg[] = {0x03, 0x00}; // Turns all I/O pins of multiplexer to output
	HAL_I2C_Master_Transmit(&hi2c2, pca, msg, 2, 10000);

	//	HAL_I2C_Master_Transmit(&hi2c2, pca, msg, 2, 10000);
	uint8_t cmd[] = {0x01, 0x00}; // Sets Pin 4 to high , red led to high
	HAL_I2C_Master_Transmit(&hi2c2, pca, cmd, 2, 10000);

	//	HAL_I2C_Master_Transmit(&hi2c2, pca, cmd, 2, 10000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);


	uint8_t veml =  0x10 << 1;
	uint8_t veml_config[] = {0x00, 0x00, 0x00};
	HAL_StatusTypeDef ret;
	HAL_I2C_Master_Transmit(&hi2c2, veml, veml_config, 3, 10000);
	if(ret != HAL_OK){
	  HAL_UART_Transmit(&huart1, "b", 1, 5000);
	}
	uint8_t cc = 0x05;
	uint16_t rgb[3];

	HAL_UART_Receive_IT(&huart1, &command, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  HAL_UART_Receive(&huart1, &command, 1, 3000);
//	  ret = HAL_I2C_Master_Transmit(&hi2c2, veml, cc, 1, 5000);
//	  if(ret != HAL_OK){
//		  HAL_UART_Transmit(&huart1, "a", 1, 5000);
//	  }
	  //Read red
	  ret = HAL_I2C_Mem_Read(&hi2c2, veml, COMMAND_VEML_RED, 1, buf, 2, 5000);
	  if(ret != HAL_OK){
		  HAL_UART_Transmit(&huart1, (void *)"Error reading veml \n", 1, 5000);
	  }
	  rgb[0] = ((uint16_t)buf[1] << 8) | buf[0];

	  //Read green
	  ret = HAL_I2C_Mem_Read(&hi2c2, veml, COMMAND_VEML_GREEN, 1, buf, 2, 5000);
	  if(ret != HAL_OK){
		  HAL_UART_Transmit(&huart1, (void *)"Error reading veml \n", 1, 5000);
	  }
	  rgb[1] = ((uint16_t)buf[1] << 8) | buf[0];

	  //Read blue
	  ret = HAL_I2C_Mem_Read(&hi2c2, veml, COMMAND_VEML_BLUE, 1, buf, 2, 5000);
	  if(ret != HAL_OK){
		  HAL_UART_Transmit(&huart1, (void *)"Error reading veml \n", 1, 5000);
	  }
	  rgb[2] = ((uint16_t)buf[1] << 8) | buf[0];

	  sprintf((char*)buf, "R,G,B: %u %u %u \n", (unsigned int)rgb[0], (unsigned int)rgb[1], (unsigned int)rgb[2]);
	  HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), 5000);
	  HAL_Delay(1000);

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//HAL_UART_Transmit(&huart1, "b", 1, 1000);
	HAL_UART_Transmit(&huart1, &command, 1, 1000);
	HAL_UART_Receive_IT(&huart1, &command, 1); //You need to toggle a breakpoint on this line!
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
