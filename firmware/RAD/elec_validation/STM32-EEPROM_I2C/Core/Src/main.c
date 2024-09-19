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
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
	RAD_TYPE_DRIVETRAIN,
	RAD_TYPE_ARM_BASE,
	RAD_TYPE_ARM_ELBOW,
	RAD_TYPE_ARM_WRIST,
	RAD_TYPE_ARM_GRIPPER,
} RAD_TYPE;


typedef struct __attribute__((packed)){
	uint8_t RAD_ID;
	RAD_TYPE RAD_TYPE;
	uint8_t HOME_POSITION;
	float P;
	float I;
	float D;
	uint32_t DRVCTRl;
	uint32_t CHOPCONF;
	uint32_t SMARTEN;
	uint32_t SGSCONF;
	uint32_t DRVCONF;
	uint16_t STEPPER_SPEED;
	uint16_t ODOM_INTERVAL;
} EEPROM_STRUCT;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define EEPROM_PAGE_TOTAL 32
#define EEPROM_PAGE_SIZE 16
#define DEVICE_IDENTIFIER 0b10100000
#define A2_PIN 0
#define A1_PIN 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

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
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */

static void READ_EEPROM(uint8_t* pData, uint32_t size, uint8_t eeprom_page_num);
static void WRITE_EEPROM(uint8_t* pData, uint32_t size, uint8_t eeprom_page_num);

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
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); //enable interrupts
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
    if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &RxHeader, RxData)
            != HAL_OK)
    {
        /* Reception Error */
        Error_Handler();
    }

    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
    // Echo received CAN message
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, echo_data, &TxMailbox);

}


// Write EEPROM Function ------------------------------

static void WRITE_EEPROM(uint8_t* pData, uint32_t size, uint8_t eeprom_page_num) {
	uint16_t eeprom_address = eeprom_page_num * EEPROM_PAGE_TOTAL;
	uint16_t pageCounter = 0;
	uint16_t address;
	uint16_t currentSize;

	uint8_t* data = (uint8_t*) malloc(EEPROM_PAGE_SIZE);

	do {
		// Setting the EEPROM address based on what page we are on
		address = eeprom_address + pageCounter * EEPROM_PAGE_SIZE;

		// Setting the first index of the data array to the EEPROM address
		data[0] = address;

		// Finding the i2c address (from validation code)
		uint8_t i2c_address = DEVICE_IDENTIFIER | (A2_PIN << 3) | (A1_PIN << 2)
			| (address >> 8) << 1 | 0;

		// Checking if the page counter is equivalent to the size passed in, divided by the EEPROM page size
		// This is integer division
		// If we are passed the last whole multiple of EEPROM Page Size, this will evaluate to true
		if (pageCounter == (size / EEPROM_PAGE_SIZE)) {
			// Calculate the number of items left to pass in, and set that to the current size
			currentSize = (size + pageCounter + 1) - pageCounter * EEPROM_PAGE_SIZE;
		} else {
			// If we are not passed the last whole multiple of EEPROM page size, set the data size to EEPROM page size
			currentSize = EEPROM_PAGE_SIZE;
		}

		// Copy elements from pData to data for use
		memcpy((void *) &data[1], (void *) &pData[pageCounter * (EEPROM_PAGE_SIZE - 1)], currentSize - 1);

		// Write data to EEPROM
		while (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) i2c_address,
				&data[0], currentSize, 10000) != HAL_OK)
		{
			/* Error_Handler() function is called when Timeout error occurs.
			 When Acknowledge failure occurs (Slave don't acknowledge its address)
			 Master restarts communication */
			if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
			{
				Error_Handler();
			}
		}

		// Incrementing the page counter
		pageCounter++;

	} while (pageCounter <= (size / EEPROM_PAGE_SIZE));
	// Deallocating memory needed
	free((void*) data);

}

static void READ_EEPROM(uint8_t* pData, uint32_t size, uint8_t eeprom_page_num) {
	uint16_t eeprom_address = eeprom_page_num * EEPROM_PAGE_TOTAL;
	uint16_t pageCounter = 0;
	uint16_t address;
	uint16_t currentSize;

	uint8_t* data = (uint8_t*) malloc(EEPROM_PAGE_SIZE - 1);

	do {
		// Setting the EEPROM address based on what page we are on
		address = eeprom_address + pageCounter * EEPROM_PAGE_SIZE;

		// Finding the i2c address (from validation code)
		uint8_t i2c_address = DEVICE_IDENTIFIER | (A2_PIN << 3) | (A1_PIN << 2)
			| (address >> 8) << 1 | 0;

		// Checking if the page counter is equivalent to the size passed in, divided by the EEPROM page size
		// This is integer division
		// If we are passed the last whole multiple of EEPROM Page Size, this will evaluate to true
		if (pageCounter == (size / EEPROM_PAGE_SIZE)) {
			// Calculate the number of items left to pass in, and set that to the current size
			currentSize = size - (pageCounter * (EEPROM_PAGE_SIZE - 1));
		} else {
			// If we are not passed the last whole multiple of EEPROM page size, set the data size to EEPROM page size
			currentSize = EEPROM_PAGE_SIZE - 1;
		}

		// READING FROM EEPROM
		while (HAL_I2C_Mem_Read(&hi2c1, (uint16_t) i2c_address,
				(uint16_t) address, I2C_MEMADD_SIZE_8BIT, &data[0],
				(uint16_t) currentSize, 10000) != HAL_OK)
		{
			/* Error_Handler() function is called when Timeout error occurs.
			 When Acknowledge failure occurs (Slave don't acknowledge its address)
			 Master restarts communication */
			if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
			{
				Error_Handler();
			}
		}

		memcpy((void *) &pData[pageCounter * (EEPROM_PAGE_SIZE - 1)], (void *) &data[0], currentSize);

		// Incrementing the page counter
		pageCounter++;

	} while (pageCounter <= (size / EEPROM_PAGE_SIZE));

	free((void*) data);
};

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
    MX_I2C1_Init();
    /* USER CODE BEGIN 2 */
    setupTxCAN();
    setupRxCAN();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        txCAN();
//        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);

        uint16_t eeprom_page = 0; // page aligned address in eeprom to write to (max 32 pages)
        EEPROM_STRUCT eeprom_data = {
        		90,
				RAD_TYPE_ARM_GRIPPER,
				180,
				0.812347,
				0.322345,
				0.523405,
				0,
				1,
				2,
				3,
				4,
				5,
				6
        };

        uint32_t eeprom_data_size = sizeof(eeprom_data) / sizeof(uint8_t);


        // write will definitely span multiple pages!
        WRITE_EEPROM((uint8_t*) &eeprom_data, eeprom_data_size, eeprom_page);

        HAL_Delay(25);

        eeprom_page = 0; // page in eeprom to read from

        uint8_t* eeprom_data_buff = (uint8_t*) malloc(eeprom_data_size);

        READ_EEPROM(eeprom_data_buff, eeprom_data_size, eeprom_page);

        EEPROM_STRUCT* eeprom_data_reconstructed = (EEPROM_STRUCT*) &eeprom_data_buff[0];

        /*
        uint8_t can_data[1];
        can_data[0] = i2c_data_buff;
        TxHeader.DLC = 1;
        HAL_CAN_AddTxMessage(&hcan, &TxHeader, can_data, &TxMailbox);
        */

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
    RCC_OscInitTypeDef RCC_OscInitStruct =
    { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct =
    { 0 };

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
    hcan.Init.Prescaler = 1;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
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
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct =
    { 0 };
    /* USER CODE BEGIN MX_GPIO_Init_1 */
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
