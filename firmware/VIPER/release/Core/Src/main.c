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
#include "ina238.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "at24c04c.h"
#include "queue.h"
#include "enc_dec_utils.h"

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

VIPER_STATE_TypeDef viper_state;
VIPER_PARAMS_TypeDef viper_params;

uint8_t ESTOP = 0;

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

	viper_state.STATE = VIPER_STATE_ACTIVE;

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

	// ##############
	// SET DRIVER INITIALIZATIONS HERE
	// todo: add all of the drivers here

	MX_AT24C04C_1_Init();
	MX_TMP_1075_Init();
	MX_MCP_3221_Init();
	MX_INA_238_Init();

	// ###############
	// MAIN INIT and BROADCAST HEALTH MESSAGE
	// VIPER_Card_Init(&viper_state, &viper_params);
	// MX_CAN_Broadcast_Health_Message(&viper_can, &viper_state);
	// ###############
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1) {

		if (!queue_empty(&can_message_queue_global)) {
			VIPER_CAN_Message_TypeDef *new_message =
					(VIPER_CAN_Message_TypeDef*) queue_front(
							&can_message_queue_global);

			switch ((int) (new_message->command_id)) {
			case ESTOP_MESSAGE: {
				ESTOP = 1;
				break;
			}
			case DISABLE_MESSAGE: {
				viper_state.STATE = VIPER_STATE_INACTIVE;
				break;
			}
			case ENABLE_MESSAGE: {
				viper_state.STATE = VIPER_STATE_ACTIVE;
				break;
			}
			case HEALTH_STATUS_PING: {
				MX_CAN_Broadcast_Health_Message(&viper_can, &viper_state);
				break;
			}
			default: {
				break;
			}
			}

			free(new_message->data);
			queue_dequeue(&can_message_queue_global);
		}

		//Only check viper queue after. This allows global messages to be addressed immediately
		else if (!queue_empty(&can_message_queue_viper)) {
			VIPER_CAN_Message_TypeDef *new_message =
					(VIPER_CAN_Message_TypeDef*) queue_front(
							&can_message_queue_viper);

			switch ((int) (new_message->command_id)) {
			case DISABLE_CARD: {
				switch (new_message->card_id){
				case VIPER_CARD_0: {
					viper_state.CARD_0.ENABLE = 0;
					break;
				}
				case VIPER_CARD_1: {
					viper_state.CARD_1.ENABLE = 0;
					break;
				}
				case VIPER_CARD_2: {
					viper_state.CARD_2.ENABLE = 0;
					break;
				}
				case VIPER_CARD_3: {
					viper_state.CARD_3.ENABLE = 0;
					break;
				}
				default:
					break;
				}
				break;
				break;
			}
			case DISABLE_ALL_CARDS: {
				viper_state.CARD_0.ENABLE = 0;
				viper_state.CARD_1.ENABLE = 0;
				viper_state.CARD_2.ENABLE = 0;
				viper_state.CARD_3.ENABLE = 0;
				break;
			}
			case ENABLE_CARD: {
				switch (new_message->card_id){
				case VIPER_CARD_0: {
					viper_state.CARD_0.ENABLE = 1;
					break;
				}
				case VIPER_CARD_1: {
					viper_state.CARD_1.ENABLE = 1;
					break;
				}
				case VIPER_CARD_2: {
					viper_state.CARD_2.ENABLE = 1;
					break;
				}
				case VIPER_CARD_3: {
					viper_state.CARD_3.ENABLE = 1;
					break;
				}
				default:
					break;
				}
				break;
			}
			case ENABLE_ALL_CARDS: {
				viper_state.CARD_0.ENABLE = 1;
				viper_state.CARD_1.ENABLE = 1;
				viper_state.CARD_2.ENABLE = 1;
				viper_state.CARD_3.ENABLE = 1;
				break;
			}
			case GET_CARD_DATA: {
				MX_CAN_Broadcast_Card_Data(&viper_can, &viper_state, new_message->card_id);
				break;
			}
			case GET_ALL_CARD_DATA: {
				MX_CAN_Broadcast_Card_Data(&viper_can, &viper_state, VIPER_CARD_0);
				MX_CAN_Broadcast_Card_Data(&viper_can, &viper_state, VIPER_CARD_1);
				MX_CAN_Broadcast_Card_Data(&viper_can, &viper_state, VIPER_CARD_2);
				MX_CAN_Broadcast_Card_Data(&viper_can, &viper_state, VIPER_CARD_3);
				break;
			}
			case SET_MUX_VALUE: {
				// todo

				TCA9544A_SelectChannel(&TCA9544A_DevAddr, channel);

				// TCA9544A takes in a pointer to device (TCA9544A HandeTypeDef( and a TCA9544A_ChannelSelect
				// Channels match the cardIDs

				// todo: need to implement a way to get the input of the channel from can
				// todo: figure out why we have a function to set the mux value when it autocycles anyways

				break;
			}
			case SAVE_TO_EEPROM: {
				// todo: Verify correctness

				ATC24C04C_WritePages(&at24c04c_1, (uint8_t*) &viper_params, sizeof(VIPER_PARAMS_TypeDef), VIPER_PARAMS_EEPROM_PAGE);
				break;
			}
			case SET_FREEZE: {
				//todo
				break;
			}
			case STOP_FREEZE: {
				//todo
				break;
			}
			// GETTERS / SETTERS
			case GET_HEALTH_INTERVAL: {
				MX_CAN_Broadcast_Uint16_Data(&viper_can, viper_params->HEALTH_INTERVAL, SEND_HEALTH_INTERVAL, 0);
				break;
			}
			case SET_HEALTH_INTERVAL: {
				viper_params->HEALTH_INTERVAL = decode_uint16_big_endian(new_message->data);
				break;
			}
			case GET_CARD_INTERVAL: {
				MX_CAN_Broadcast_Uint16_Data(&viper_can, viper_params->CARD_INTERVAL, SEND_CARD_INTERVAL, 0);
				break;
			}
			case SET_CARD_INTERVAL: {
				viper_params->CARD_INTERVAL = decode_uint16_big_endian(new_message->data);
				break;
			}
			default:
				break;
			}
			free(new_message->data);
			queue_dequeue(&can_message_queue_viper);
		}

		VIPER_Card_Update_State(&viper_state);
		VIPER_Card_Update_Params(&viper_state, &viper_params);

		if (ESTOP) {
			HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
			break;
		}

		// MAIN STATE MACHINE START
		switch (viper_state.STATE) {
		case VIPER_STATE_INACTIVE: {
			HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
			break;
		}
		case VIPER_STATE_ACTIVE: {
			HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

			VIPER_Card_Read(&viper_state, viper_state.CURRENT_CARD);

			break;
		}
		default:
			break;
		}

		// SCHEDULED TASKS
		if ((viper_params.CARD_INTERVAL != 0)
				&& (HAL_GetTick() % viper_params.CARD_INTERVAL == 0)) {
			MX_CAN_Broadcast_Card_Data(&viper_can, &viper_state, viper_state.CURRENT_CARD);
		}

		if ((viper_params.HEALTH_INTERVAL != 0)
				&& (HAL_GetTick() % viper_params.HEALTH_INTERVAL == 0)) {
			MX_CAN_Broadcast_Health_Message(&viper_can, &viper_state);
		}

		// MAIN STATE MACHINE END
		switch (viper_state.STATE) {
		case VIPER_STATE_INACTIVE: {
			break;
		}
		case VIPER_STATE_ACTIVE: {
			viper_state.CURRENT_CARD++;
			if (viper_state.CURRENT_CARD == 4)
				viper_state.CURRENT_CARD = VIPER_CARD_0;

			break;
		}
		default:
			break;
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

void VIPER_Card_Init(VIPER_STATE_TypeDef *viper_state, VIPER_PARAMS_TypeDef *viper_params) {
	// Step 1: initialize viper_params (from EEPROM)
	AT24C04C_ReadPages(&at24c04c_1, (uint8_t*) &viper_params, sizeof(VIPER_PARAMS_TypeDef), VIPER_PARAMS_EEPROM_PAGE);

	// Step 2: set any relevant fields in viper_state from viper_params
	viper_state->CARD_0.ENABLE = viper_params->CARD_0.ENABLE;
	viper_state->CARD_1.ENABLE = viper_params->CARD_1.ENABLE;
	viper_state->CARD_2.ENABLE = viper_params->CARD_2.ENABLE;
	viper_state->CARD_3.ENABLE = viper_params->CARD_3.ENABLE;

	// Step 3: Setup VIPER
	VIPER_Card_Update_State(viper_state);
}

void VIPER_Card_Check(VIPER_STATE_TypeDef *viper_state) {
	uint8_t pgood[4];
	uint8_t pgood[0] = (uint8_t) HAL_GPIO_ReadPin(PGOOD_CARD_0_GPIO_Port,
			PGOOD_CARD_0_Pin); // CARD 0
	uint8_t pgood[1] = (uint8_t) HAL_GPIO_ReadPin(PGOOD_CARD_1_GPIO_Port,
			PGOOD_CARD_1_Pin); // CARD 1
	uint8_t pgood[2] = (uint8_t) HAL_GPIO_ReadPin(PGOOD_CARD_2_GPIO_Port,
			PGOOD_CARD_2_Pin); // CARD 2
	uint8_t pgood[3] = (uint8_t) HAL_GPIO_ReadPin(PGOOD_CARD_3_GPIO_Port,
			PGOOD_CARD_3_Pin); // CARD 3

	uint8_t infault[4];
	uint8_t infault[0] = (uint8_t) HAL_GPIO_ReadPin(IN_FAULT_0_GPIO_Port,
			IN_FAULT_0_Pin); // CARD 0
	uint8_t infault[1] = (uint8_t) HAL_GPIO_ReadPin(IN_FAULT_1_GPIO_Port,
			IN_FAULT_1_Pin); // CARD 1
	uint8_t infault[2] = (uint8_t) HAL_GPIO_ReadPin(IN_FAULT_2_GPIO_Port,
			IN_FAULT_2_Pin); // CARD 2
	uint8_t infault[3] = (uint8_t) HAL_GPIO_ReadPin(IN_FAULT_3_GPIO_Port,
			IN_FAULT_3_Pin); // CARD 3

	uint8_t outfault[6];
	uint8_t outfault[0] = (uint8_t) HAL_GPIO_ReadPin(OUT_FAULT_0_GPIO_Port,
			OUT_FAULT_0_Pin); // CARD 0 : LP
	uint8_t outfault[1] = (uint8_t) HAL_GPIO_ReadPin(OUT_FAULT_1_GPIO_Port,
				OUT_FAULT_1_Pin); // CARD 0 : LP
	uint8_t outfault[2] = (uint8_t) HAL_GPIO_ReadPin(OUT_FAULT_2_GPIO_Port,
			OUT_FAULT_2_Pin); // CARD 1 : HP
	uint8_t outfault[3] = (uint8_t) HAL_GPIO_ReadPin(OUT_FAULT_3_GPIO_Port,
			OUT_FAULT_3_Pin); // CARD 2 : HP
	uint8_t outfault[4] = (uint8_t) HAL_GPIO_ReadPin(OUT_FAULT_4_GPIO_Port,
			OUT_FAULT_4_Pin); // CARD 3 : LP
	uint8_t outfault[5] = (uint8_t) HAL_GPIO_ReadPin(OUT_FAULT_5_GPIO_Port,
				OUT_FAULT_5_Pin); // CARD 3 : LP

	viper_state->CARD_0.CONNECTED = pgood[0];
	viper_state->CARD_1.CONNECTED = pgood[1];
	viper_state->CARD_2.CONNECTED = pgood[2];
	viper_state->CARD_3.CONNECTED = pgood[3];

	viper_state->CARD_0.INPUT_FAULT = !infault[0];
	viper_state->CARD_1.INPUT_FAULT = !infault[1];
	viper_state->CARD_2.INPUT_FAULT = !infault[2];
	viper_state->CARD_3.INPUT_FAULT = !infault[3];

	viper_state->CARD_0.OUTPUT_FAULT_A = outfault[0];
	viper_state->CARD_0.OUTPUT_FAULT_B = outfault[1];
	viper_state->CARD_1.OUTPUT_FAULT_A = outfault[2];
	viper_state->CARD_2.OUTPUT_FAULT_A = outfault[3];
	viper_state->CARD_3.OUTPUT_FAULT_A = outfault[4];
	viper_state->CARD_3.OUTPUT_FAULT_B = outfault[5];

	if (!viper_state->CARD_0.CONNECTED) {
		viper_state->CARD_0.STATUS = VIPER_CARD_DISCONNECTED;
	} else if (viper_state->CARD_0.INPUT_FAULT || viper_state->CARD_0.OUTPUT_FAULT_A || viper_state->CARD_0.OUTPUT_FAULT_B) {
		viper_state->CARD_0.STATUS = VIPER_CARD_FAULT;
	} else {
		viper_state->CARD_0.STATUS = VIPER_CARD_OK;
	}

	if (!viper_state->CARD_1.CONNECTED) {
		viper_state->CARD_1.STATUS = VIPER_CARD_DISCONNECTED;
	} else if (viper_state->CARD_1.INPUT_FAULT || viper_state->CARD_1.OUTPUT_FAULT_A) {
		viper_state->CARD_1.STATUS = VIPER_CARD_FAULT;
	} else {
		viper_state->CARD_1.STATUS = VIPER_CARD_OK;
	}

	if (!viper_state->CARD_2.CONNECTED) {
		viper_state->CARD_2.STATUS = VIPER_CARD_DISCONNECTED;
	} else if (viper_state->CARD_2.INPUT_FAULT || viper_state->CARD_2.OUTPUT_FAULT_A) {
		viper_state->CARD_2.STATUS = VIPER_CARD_FAULT;
	} else {
		viper_state->CARD_2.STATUS = VIPER_CARD_OK;
	}

	if (!viper_state->CARD_3.CONNECTED) {
		viper_state->CARD_3.STATUS = VIPER_CARD_DISCONNECTED;
	} else if (viper_state->CARD_3.INPUT_FAULT || viper_state->CARD_3.OUTPUT_FAULT_A || viper_state->CARD_3.OUTPUT_FAULT_B) {
		viper_state->CARD_3.STATUS = VIPER_CARD_FAULT;
	} else {
		viper_state->CARD_3.STATUS = VIPER_CARD_OK;
	}
}

void VIPER_Card_Update_Params(VIPER_STATE_TypeDef *viper_state, VIPER_PARAMS_TypeDef *viper_params) {
	// STEP 1: Update relevant fields in viper_params (stored in EEPROM) from viper_state
	// i.e. data which is both DYNAMIC and PERSISTENT
	viper_params->CARD_0.ENABLE = viper_state->CARD_0.ENABLE;
	viper_params->CARD_1.ENABLE = viper_state->CARD_1.ENABLE;
	viper_params->CARD_2.ENABLE = viper_state->CARD_2.ENABLE;
	viper_params->CARD_3.ENABLE = viper_state->CARD_3.ENABLE;

	// STEP 2: Update EEPROM from viper_params
	AT24C04C_WritePages(&at24c04c_1, (uint8_t*) &viper_params, sizeof(VIPER_PARAMS_TypeDef), VIPER_PARAMS_EEPROM_PAGE);
}

void VIPER_Card_Update_State(VIPER_STATE_TypeDef *viper_state) {
	// Any updates that viper_state should do after being changed
	if (viper_state->CARD_0.ENABLE)
		HAL_GPIO_WritePin(EN_CARD_0_GPIO_Port, EN_CARD_0_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(EN_CARD_0_GPIO_Port, EN_CARD_0_Pin, GPIO_PIN_RESET);

	if (viper_state->CARD_1.ENABLE)
		HAL_GPIO_WritePin(EN_CARD_1_GPIO_Port, EN_CARD_1_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(EN_CARD_1_GPIO_Port, EN_CARD_1_Pin, GPIO_PIN_RESET);

	if (viper_state->CARD_2.ENABLE)
		HAL_GPIO_WritePin(EN_CARD_2_GPIO_Port, EN_CARD_2_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(EN_CARD_2_GPIO_Port, EN_CARD_2_Pin, GPIO_PIN_RESET);

	if (viper_state->CARD_3.ENABLE)
		HAL_GPIO_WritePin(EN_CARD_3_GPIO_Port, EN_CARD_3_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(EN_CARD_3_GPIO_Port, EN_CARD_3_Pin, GPIO_PIN_RESET);
}

void VIPER_Card_Read(VIPER_STATE_TypeDef* viper_state, VIPER_CARD_ID_TypeDef cardx) {
	// todo: read all relevant diagnostic values using drivers

	//todo: Figure out ADC incorporation (MCP driver)

	// todo: Fix addresses

	// todo: Make sure the correct values are being written

	// 1. Select the relevant card with the mux

	TCA9544A_SelectChannel(&TCA9544A_DevAddr, cardx); // Todo: confirm this address, seen as it's probably incorrect

	// 2. Read the temperature

	TMP_1075_ReadTemp(&tmp_1075);
	TCA9544A_ReadChannel(&TCA9544A_DevAddr);
	switch (cardx) {
		case VIPER_CARD_0: {

			viper_state->CARD_0.TEMPERATURE = tmp_1075->temp;
			break;
		}
		case VIPER_CARD_1: {

			viper_state->CARD_1.TEMPERATURE = tmp_1075->temp;
			break;

		}
		case VIPER_CARD_2: {

			viper_state->CARD_2.TEMPERATURE = tmp_1075->temp;
			break;
		}

		case VIPER_CARD_3:  {
			viper_state->CARD_3.TEMPERATURE = tmp_1075->temp;
			break;
		}
		default: {
			break;
		}


	}

	// 2. Read the Current


	INA_238_ReadCurrent(&ina238_carda);
	if (cardx == VIPER_CARD_0 || cardx == VIPER_CARD_3) {
		INA_238_ReadCurrent(&ina238_cardb);
	}
	TCA9544A_ReadChannel(&TCA9544A_DevAddr);
	switch (cardx) {
		case VIPER_CARD_0: {

			viper_state->CARD_0.OUTPUT_CURRENT_A = ina238_carda->current;
			viper_state->CARD_0.OUTPUT_CURRENT_B = ina238_cardb->current;
			break;
		}
		case VIPER_CARD_1: {

			viper_state->CARD_1.OUTPUT_CURRENT_A = ina238_carda->current;
			break;

		}
		case VIPER_CARD_2: {

			viper_state->CARD_2.OUTPUT_CURRENT_A = ina238_carda->current;
			break;
		}

		case VIPER_CARD_3:  {
			viper_state->CARD_3.OUTPUT_CURRENT_A = ina238_carda->current;
			viper_state->CARD_3.OUTPUT_CURRENT_B = ina238_cardb->current;
			break;
		}
		default: {
			break;
		}




	}

	// 3. Read the Voltage

	// This is the same idea as current, where two of them need to be checked

	INA_238_ReadVoltage(&ina238_carda);
	if (cardx == VIPER_CARD_0 || cardx == VIPER_CARD_3) {
		INA_238_ReadVoltage(&ina238_cardb);
	}
	TCA9544A_ReadChannel(&TCA9544A_DevAddr);
	switch (cardx) {
		case VIPER_CARD_0: {

			viper_state->CARD_0.OUTPUT_VOLTAGE_A = ina238_carda->voltage;
			viper_state->CARD_0.OUTPUT_VOLTAGE_B = ina238_cardb->voltage;
			break;
		}
		case VIPER_CARD_1: {

			viper_state->CARD_1.OUTPUT_VOLTAGE_A = ina238_carda->voltage;
			break;

		}
		case VIPER_CARD_2: {

			viper_state->CARD_2.OUTPUT_VOLTAGE_A = ina238_carda->voltage;
			break;
		}

		case VIPER_CARD_3:  {
			viper_state->CARD_3.OUTPUT_VOLTAGE_A = ina238_carda->voltage;
			viper_state->CARD_3.OUTPUT_VOLTAGE_B = ina238_cardb->voltage;
			break;
		}
		default: {
			break;
		}




	}

	// 4. Read the Power

	INA_238_ReadPower(&ina238_carda);
	if (cardx == VIPER_CARD_0 || cardx == VIPER_CARD_3) {
		INA_238_ReadPower(&ina238_cardb);
	}
	TCA9544A_ReadChannel(&TCA9544A_DevAddr);
	switch (cardx) {
		case VIPER_CARD_0: {

			viper_state->CARD_0.OUTPUT_POWER_A = ina238_carda->power;
			viper_state->CARD_0.OUTPUT_POWER_B = ina238_cardb->power;
			break;
		}
		case VIPER_CARD_1: {

			viper_state->CARD_1.OUTPUT_POWER_A = ina238_carda->power;
			break;

		}
		case VIPER_CARD_2: {

			viper_state->CARD_2.OUTPUT_POWER_A = ina238_carda->power;
			break;
		}

		case VIPER_CARD_3:  {
			viper_state->CARD_3.OUTPUT_POWER_A = ina238_carda->power;
			viper_state->CARD_3.OUTPUT_POWER_B = ina238_cardb->power;
			break;
		}
		default: {
			break;
		}




	}


	// This is the exact same idea, where the low power cards will have two powers






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
	while (1) {
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
