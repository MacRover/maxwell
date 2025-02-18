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
#include "ina238.h"
#include "tca.h"
#include "tmp.h"
#include "viper_mcp3221.h"

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
uint8_t FREEZE = 0;

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
	viper_params.VIPER_ID = 0x01;
	viper_params.HEALTH_INTERVAL = 1000; //every second
	viper_params.CARD_INTERVAL = 1000; // may need to tune this

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

	MX_AT24C04C_1_Init();
	MX_TMP_1075_Init();
	MX_MCP_3221_Init();
	MX_INA_238_Init();
	MX_TCA_Init();

	// ###############
	// MAIN INIT and BROADCAST HEALTH MESSAGE
	VIPER_Card_Init(&viper_state, &viper_params);
	MX_CAN_Broadcast_Health_Message(&viper_can, &viper_state);
	// ###############


	TCA9544A_SelectChannel(&tca, VIPER_CARD_0);

	INA_238_WriteConfig(&ina238_low_power_a);
    INA_238_WriteConfig(&ina238_low_power_b);

	TMP_1075_SetHighLimit(&h_tmp_1075);
    TMP_1075_SetLowLimit(&h_tmp_1075);
    TMP_1075_SetConfRegisters(&h_tmp_1075);

	TCA9544A_SelectChannel(&tca, VIPER_CARD_1);

	INA_238_WriteConfig(&ina238_high_power);


	TMP_1075_SetHighLimit(&h_tmp_1075);
    TMP_1075_SetLowLimit(&h_tmp_1075);
    TMP_1075_SetConfRegisters(&h_tmp_1075);

	TCA9544A_SelectChannel(&tca, VIPER_CARD_2);

	INA_238_WriteConfig(&ina238_high_power);


	TMP_1075_SetHighLimit(&h_tmp_1075);
    TMP_1075_SetLowLimit(&h_tmp_1075);
    TMP_1075_SetConfRegisters(&h_tmp_1075);

	TCA9544A_SelectChannel(&tca, VIPER_CARD_3);

	INA_238_WriteConfig(&ina238_low_power_a);
    INA_238_WriteConfig(&ina238_low_power_b);

	TMP_1075_SetHighLimit(&h_tmp_1075);
    TMP_1075_SetLowLimit(&h_tmp_1075);
    TMP_1075_SetConfRegisters(&h_tmp_1075);

	TCA9544A_SelectChannel(&tca, VIPER_CARD_0);

    

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	// ESTOP --> DISABLE --> FREEZE -> ACTIVE

	while (1) {

		//VIPER_Card_Check(&viper_state);

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
				TCA9544A_SelectChannel(&tca, new_message->card_id);
				viper_state.CURRENT_CARD = new_message->card_id;
				break;
			}
			case SAVE_TO_EEPROM: {
				AT24C04C_WritePages(&at24c04c_1, (uint8_t*) &viper_params, sizeof(VIPER_PARAMS_TypeDef), VIPER_PARAMS_EEPROM_PAGE);
				break;
			}
			case SET_FREEZE: {
				FREEZE = 1;
				break;
			}
			case STOP_FREEZE: {
				FREEZE = 0;
				break;
			}
			// GETTERS / SETTERS
			case GET_HEALTH_INTERVAL: {
				MX_CAN_Broadcast_Uint32_Data(&viper_can, viper_params.HEALTH_INTERVAL, GET_HEALTH_INTERVAL, 0);
				break;
			}
			case SET_HEALTH_INTERVAL: {
				viper_params.HEALTH_INTERVAL = decode_uint32_big_endian(new_message->data);
				break;
			}
			case GET_CARD_INTERVAL: {
				MX_CAN_Broadcast_Uint32_Data(&viper_can, viper_params.CARD_INTERVAL, GET_CARD_INTERVAL, 0);
				break;
			}
			case SET_CARD_INTERVAL: {
				viper_params.CARD_INTERVAL = decode_uint32_big_endian(new_message->data);
				break;
			}
			default:
				break;
			}
			free(new_message->data);
			queue_dequeue(&can_message_queue_viper);
		}

		 VIPER_Card_Update_State(&viper_state);
		// VIPER_Card_Update_Params(&viper_state, &viper_params);

		if (ESTOP) {
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			break;
		}

		// MAIN STATE MACHINE START
		switch (viper_state.STATE) {
		case VIPER_STATE_INACTIVE: {
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			break;
		}
		case VIPER_STATE_ACTIVE: {
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);


			if (!FREEZE) {
				if ((viper_params.CARD_INTERVAL != 0) && (HAL_GetTick() % viper_params.CARD_INTERVAL == 0)) {

					if (viper_state.CURRENT_CARD == VIPER_CARD_3) {
						viper_state.CURRENT_CARD = VIPER_CARD_0;
					} else {
						viper_state.CURRENT_CARD++;
					}
				}
			}

			if((viper_state.CURRENT_CARD == VIPER_CARD_0 && viper_state.CARD_0.ENABLE)
			|| (viper_state.CURRENT_CARD == VIPER_CARD_1 && viper_state.CARD_1.ENABLE)
			|| (viper_state.CURRENT_CARD == VIPER_CARD_2 && viper_state.CARD_2.ENABLE)
			|| (viper_state.CURRENT_CARD == VIPER_CARD_3 && viper_state.CARD_3.ENABLE))
			{
				VIPER_Card_Read(&viper_state, viper_state.CURRENT_CARD);
			}
			
			break;
		}
		default:
			break;
		}

		// SCHEDULED TASKS
		if ((viper_params.CARD_INTERVAL != 0) && (HAL_GetTick() % viper_params.CARD_INTERVAL == 0)) {
			MX_CAN_Broadcast_Card_Data(&viper_can, &viper_state, viper_state.CURRENT_CARD);
		}

		// if ((viper_params.HEALTH_INTERVAL != 0)
		// 		&& (HAL_GetTick() % viper_params.HEALTH_INTERVAL == 0)) {
		// 	MX_CAN_Broadcast_Health_Message(&viper_can, &viper_state);
		// }

		// MAIN STATE MACHINE END

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
	// AT24C04C_ReadPages(&at24c04c_1, (uint8_t*) &viper_params, sizeof(VIPER_PARAMS_TypeDef), VIPER_PARAMS_EEPROM_PAGE);

	// Step 2: set any relevant fields in viper_state from viper_params
//	viper_state->CARD_0.ENABLE = viper_params->CARD_0.ENABLE;
//	viper_state->CARD_1.ENABLE = viper_params->CARD_1.ENABLE;
//	viper_state->CARD_2.ENABLE = viper_params->CARD_2.ENABLE;
//	viper_state->CARD_3.ENABLE = viper_params->CARD_3.ENABLE;

	viper_state->CARD_0.ENABLE = 1;
	viper_state->CARD_1.ENABLE = 1;
	viper_state->CARD_2.ENABLE = 1;
	viper_state->CARD_3.ENABLE = 1;

	// Step 3: Setup VIPER
	VIPER_Card_Update_State(viper_state);
}

void VIPER_Card_Check(VIPER_STATE_TypeDef *viper_state) {
	uint8_t c0 = viper_state->CARD_0.ENABLE;
	uint8_t c1 = viper_state->CARD_1.ENABLE;
	uint8_t c2 = viper_state->CARD_2.ENABLE;
	uint8_t c3 = viper_state->CARD_3.ENABLE;

	uint8_t pgood[4] = {0};
	uint8_t infault[4] = {0};
	uint8_t outfault[6] = {0};

	if (1) {
		pgood[0] = (uint8_t) HAL_GPIO_ReadPin(PGOOD_CARD_0_GPIO_Port,
					PGOOD_CARD_0_Pin); // CARD 0
		infault[0] = (uint8_t) HAL_GPIO_ReadPin(IN_FAULT_CARD_0_GPIO_Port,
					IN_FAULT_CARD_0_Pin); // CARD 0
		outfault[0] = (uint8_t) HAL_GPIO_ReadPin(OUT_FAULT_0_GPIO_Port,
					OUT_FAULT_0_Pin); // CARD 0 : LP
		outfault[1] = (uint8_t) HAL_GPIO_ReadPin(OUT_FAULT_1_GPIO_Port,
					OUT_FAULT_1_Pin); // CARD 0 : LP
		viper_state->CARD_0.CONNECTED = pgood[0];
		viper_state->CARD_0.INPUT_FAULT = !infault[0];
		viper_state->CARD_0.OUTPUT_FAULT_A = outfault[0];
		viper_state->CARD_0.OUTPUT_FAULT_B = outfault[1];
	}

	if (1) {
		pgood[1] = (uint8_t) HAL_GPIO_ReadPin(PGOOD_CARD_1_GPIO_Port,
					PGOOD_CARD_1_Pin); // CARD 1
		infault[1] = (uint8_t) HAL_GPIO_ReadPin(IN_FAULT_CARD_1_GPIO_Port,
					IN_FAULT_CARD_1_Pin); // CARD 1
		outfault[2] = (uint8_t) HAL_GPIO_ReadPin(OUT_FAULT_2_GPIO_Port,
					OUT_FAULT_2_Pin); // CARD 1 : HP
		viper_state->CARD_1.CONNECTED = pgood[1];
		viper_state->CARD_1.INPUT_FAULT = !infault[1];
		viper_state->CARD_1.OUTPUT_FAULT_A = outfault[2];
	}

	if (1) {
		pgood[2] = (uint8_t) HAL_GPIO_ReadPin(PGOOD_CARD_2_GPIO_Port,
					PGOOD_CARD_2_Pin); // CARD 2
		infault[2] = (uint8_t) HAL_GPIO_ReadPin(IN_FAULT_CARD_2_GPIO_Port,
					IN_FAULT_CARD_2_Pin); // CARD 2
		outfault[3] = (uint8_t) HAL_GPIO_ReadPin(OUT_FAULT_3_GPIO_Port,
					OUT_FAULT_3_Pin); // CARD 2 : HP
		viper_state->CARD_2.CONNECTED = pgood[2];
		viper_state->CARD_2.INPUT_FAULT = !infault[2];
		viper_state->CARD_2.OUTPUT_FAULT_A = outfault[3];
	}

	if (1) {
		pgood[3] = (uint8_t) HAL_GPIO_ReadPin(PGOOD_CARD_3_GPIO_Port,
					PGOOD_CARD_3_Pin); // CARD 3
		infault[3] = (uint8_t) HAL_GPIO_ReadPin(IN_FAULT_CARD_3_GPIO_Port,
					IN_FAULT_CARD_3_Pin); // CARD 3
		outfault[4] = (uint8_t) HAL_GPIO_ReadPin(OUT_FAULT_4_GPIO_Port,
					OUT_FAULT_4_Pin); // CARD 3 : LP
		outfault[5] = (uint8_t) HAL_GPIO_ReadPin(OUT_FAULT_5_GPIO_Port,
						OUT_FAULT_5_Pin); // CARD 3 : LP
		viper_state->CARD_3.CONNECTED = pgood[3];
		viper_state->CARD_3.INPUT_FAULT = !infault[3];
		viper_state->CARD_3.OUTPUT_FAULT_A = outfault[4];
		viper_state->CARD_3.OUTPUT_FAULT_B = outfault[5];
	}

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

//	if (c0) {
//		if (viper_state->CARD_0.STATUS != VIPER_CARD_OK) {
//			viper_state->CARD_0.ENABLE = 0;
//		}
//	}
//
//	if (c1) {
//		if (viper_state->CARD_1.STATUS != VIPER_CARD_OK) {
//			viper_state->CARD_1.ENABLE = 0;
//		}
//	}
//
//	if (c2) {
//		if (viper_state->CARD_2.STATUS != VIPER_CARD_OK) {
//			viper_state->CARD_2.ENABLE = 0;
//		}
//	}
//
//	if (c3) {
//		if (viper_state->CARD_3.STATUS != VIPER_CARD_OK) {
//			viper_state->CARD_3.ENABLE = 0;
//		}
//	}
}

void VIPER_Card_Update_Params(VIPER_STATE_TypeDef *viper_state, VIPER_PARAMS_TypeDef *viper_params) {
	// STEP 1: Update relevant fields in viper_params (stored in EEPROM) from viper_state
	// i.e. data which is both DYNAMIC and PERSISTENT
	uint8_t changed_flag = VIPER_Card_Params_Flag(viper_state, viper_params);

	// STEP 2: Update EEPROM from viper_params
	if (changed_flag)
	{
		// AT24C04C_WritePages(&at24c04c_1, (uint8_t*) &viper_params, sizeof(VIPER_PARAMS_TypeDef), VIPER_PARAMS_EEPROM_PAGE);
	}
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
	// 1. Select the relevant card with the mux

	TCA9544A_SelectChannel(&tca, cardx);

	// 2. Read the temperature

	//TMP_1075_ReadTemp(&h_tmp_1075);

	// 2. Read the rest

	if (cardx == VIPER_CARD_0 || cardx == VIPER_CARD_3) {
		INA_238_ReadCurrent(&ina238_low_power_a);
		INA_238_ReadCurrent(&ina238_low_power_b);

		INA_238_ReadVoltage(&ina238_low_power_a);
		INA_238_ReadVoltage(&ina238_low_power_b);

		// INA_238_ReadPower(&ina238_low_power_a);
		// INA_238_ReadPower(&ina238_low_power_b);

		// INA_238_ReadDiagnostic(&ina238_low_power_a);
		// INA_238_ReadDiagnostic(&ina238_low_power_b);

		//MCP3221_ReadCurrent(&input_current_low_card);
	} else {

		INA_238_ReadCurrent(&ina238_high_power);
		INA_238_ReadVoltage(&ina238_high_power);
		//INA_238_ReadPower(&ina238_high_power);
		// INA_238_ReadDiagnostic(&ina238_high_power);

		MCP3221_ReadCurrent(&input_current_high_card);
	}

	// Step 3: Writing everything

	switch (cardx) {
		case VIPER_CARD_0: {

			//viper_state->CARD_0.TEMPERATURE = h_tmp_1075.temp;
			//viper_state->CARD_0.INPUT_CURRENT = input_current_low_card.current/1000;
			viper_state->CARD_0.OUTPUT_CURRENT_A = ina238_low_power_a.current;
			viper_state->CARD_0.OUTPUT_CURRENT_B = ina238_low_power_b.current;
			viper_state->CARD_0.OUTPUT_VOLTAGE_A = ina238_low_power_a.voltage;
			viper_state->CARD_0.OUTPUT_VOLTAGE_B = ina238_low_power_b.voltage;
			// viper_state->CARD_0.OUTPUT_POWER_A = ina238_low_power_a.power;
			// viper_state->CARD_0.OUTPUT_POWER_B = ina238_low_power_b.power;
			// viper_state->CARD_0.OUTPUT_DIAGNOSTIC_A = ina238_low_power_a.diagnostic;
			// viper_state->CARD_0.OUTPUT_DIAGNOSTIC_B = ina238_low_power_b.diagnostic;
			break;
		}
		case VIPER_CARD_1: {

			//viper_state->CARD_1.TEMPERATURE = h_tmp_1075.temp;
			viper_state->CARD_1.INPUT_CURRENT = input_current_high_card.current/1000;
			viper_state->CARD_1.OUTPUT_CURRENT_A = ina238_high_power.current;
			viper_state->CARD_1.OUTPUT_VOLTAGE_A = ina238_high_power.voltage;
			// viper_state->CARD_1.OUTPUT_POWER_A = ina238_high_power.power;
			// viper_state->CARD_1.OUTPUT_DIAGNOSTIC_A = ina238_high_power.diagnostic;
			break;

		}
		case VIPER_CARD_2: {

			//viper_state->CARD_2.TEMPERATURE = h_tmp_1075.temp;
			viper_state->CARD_2.INPUT_CURRENT = input_current_high_card.current/1000;
			viper_state->CARD_2.OUTPUT_CURRENT_A = ina238_high_power.current;
			viper_state->CARD_2.OUTPUT_VOLTAGE_A = ina238_high_power.voltage;
			// viper_state->CARD_2.OUTPUT_POWER_A = ina238_high_power.power;
			// viper_state->CARD_2.OUTPUT_DIAGNOSTIC_A = ina238_high_power.diagnostic;
			break;
		}

		case VIPER_CARD_3:  {

			// viper_state->CARD_3.TEMPERATURE = h_tmp_1075.temp;
			// viper_state->CARD_3.INPUT_CURRENT = input_current_low_card.current/1000;
			viper_state->CARD_3.OUTPUT_CURRENT_A = ina238_low_power_a.current;
			viper_state->CARD_3.OUTPUT_CURRENT_B = ina238_low_power_b.current;
			viper_state->CARD_3.OUTPUT_VOLTAGE_A = ina238_low_power_a.voltage;
			viper_state->CARD_3.OUTPUT_VOLTAGE_B = ina238_low_power_b.voltage;
			// viper_state->CARD_3.OUTPUT_POWER_A = ina238_low_power_a.power;
			// viper_state->CARD_3.OUTPUT_POWER_B = ina238_low_power_b.power;
			// viper_state->CARD_3.OUTPUT_DIAGNOSTIC_A = ina238_low_power_a.diagnostic;
			// viper_state->CARD_3.OUTPUT_DIAGNOSTIC_B = ina238_low_power_b.diagnostic;
			break;
		}
		default: {
			break;
		}
	}
	// This is the exact same idea, where the low power cards will have two powers
}

uint8_t VIPER_Card_Params_Flag(VIPER_STATE_TypeDef* viper_state, VIPER_PARAMS_TypeDef *viper_params) {
	// Step 1: check which fields have changed

	uint8_t changed_flag = 0;

	if (viper_params->CARD_0.ENABLE != viper_state->CARD_0.ENABLE) {
		viper_params->CARD_0.ENABLE = viper_state->CARD_0.ENABLE;
		changed_flag = 1;
	}

	if (viper_params->CARD_1.ENABLE != viper_state->CARD_1.ENABLE) {
		viper_params->CARD_1.ENABLE = viper_state->CARD_1.ENABLE;
		changed_flag = 1;
	}

	if (viper_params->CARD_2.ENABLE != viper_state->CARD_2.ENABLE) {
		viper_params->CARD_2.ENABLE = viper_state->CARD_2.ENABLE;
		changed_flag = 1;
	}

	if (viper_params->CARD_3.ENABLE != viper_state->CARD_3.ENABLE) {
		viper_params->CARD_3.ENABLE = viper_state->CARD_3.ENABLE;
		changed_flag = 1;
	}

	return changed_flag;
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
