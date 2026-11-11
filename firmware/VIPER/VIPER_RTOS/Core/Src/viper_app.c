/*
 * viper_app.c
 *
 *  Created on: Oct 23, 2026
 *      Author: Manil Ferr
 */

#include <stdlib.h>

#include "viper_queue.h"
#include "at24c04c.h"
#include "viper_app.h"
#include "enc_dec_utils.h"
#include "ina238.h"
#include "tca.h"
#include "tmp.h"
#include "viper_mcp3221.h"
#include "can.h"

#ifdef VIPER_TEST
// Keep reports beside the operations when moving them into RTOS tasks
#define TEST_OUTPUT(command, card) test_output_command = (command); test_output_card = (card)
#define TEST_RESULT(call) test_result = (call)
static uint32_t test_output_mask(void)
{
    return HAL_GPIO_ReadPin(EN_CARD_0_GPIO_Port, EN_CARD_0_Pin)
        | (HAL_GPIO_ReadPin(EN_CARD_1_GPIO_Port, EN_CARD_1_Pin) << 1)
        | (HAL_GPIO_ReadPin(EN_CARD_2_GPIO_Port, EN_CARD_2_Pin) << 2)
        | (HAL_GPIO_ReadPin(EN_CARD_3_GPIO_Port, EN_CARD_3_Pin) << 3);
}
#else
#define TEST_OUTPUT(command, card) ((void)0)
#define TEST_RESULT(call) ((void)(call))
#endif

void viper_mainInit(void){
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

	TMP_100_SetHighLimit(&h_tmp_100_a);
    TMP_100_SetLowLimit(&h_tmp_100_a);
    TMP_100_SetConfRegisters(&h_tmp_100_a);

    TMP_100_SetHighLimit(&h_tmp_100_b);
    TMP_100_SetLowLimit(&h_tmp_100_b);
    TMP_100_SetConfRegisters(&h_tmp_100_b);

	

	TCA9544A_SelectChannel(&tca, VIPER_CARD_1);

	INA_238_WriteConfig(&ina238_high_power);

	TMP_1075_SetHighLimit(&h_tmp_1075);
    TMP_1075_SetLowLimit(&h_tmp_1075);
    TMP_1075_SetConfRegisters(&h_tmp_1075);

	TMP_100_SetHighLimit(&h_tmp_100_a);
    TMP_100_SetLowLimit(&h_tmp_100_a);
    TMP_100_SetConfRegisters(&h_tmp_100_a);

    TMP_100_SetHighLimit(&h_tmp_100_b);
    TMP_100_SetLowLimit(&h_tmp_100_b);
    TMP_100_SetConfRegisters(&h_tmp_100_b);



	TCA9544A_SelectChannel(&tca, VIPER_CARD_2);

	INA_238_WriteConfig(&ina238_high_power);

	TMP_1075_SetHighLimit(&h_tmp_1075);
    TMP_1075_SetLowLimit(&h_tmp_1075);
    TMP_1075_SetConfRegisters(&h_tmp_1075);

	TMP_100_SetHighLimit(&h_tmp_100_a);
    TMP_100_SetLowLimit(&h_tmp_100_a);
    TMP_100_SetConfRegisters(&h_tmp_100_a);

    //TMP_100_SetHighLimit(&h_tmp_100_b);
    //TMP_100_SetLowLimit(&h_tmp_100_b);
    //TMP_100_SetConfRegisters(&h_tmp_100_b);


	//NO CARD 3 at the moment!
	// TCA9544A_SelectChannel(&tca, VIPER_CARD_3);

	// INA_238_WriteConfig(&ina238_low_power_a);
    // INA_238_WriteConfig(&ina238_low_power_b);

	// TMP_1075_SetHighLimit(&h_tmp_1075);
    // TMP_1075_SetLowLimit(&h_tmp_1075);
    // TMP_1075_SetConfRegisters(&h_tmp_1075);

	// TMP_100_SetHighLimit(&h_tmp_100_a);
    // TMP_100_SetLowLimit(&h_tmp_100_a);
    // TMP_100_SetConfRegisters(&h_tmp_100_a);

    // TMP_100_SetHighLimit(&h_tmp_100_b);
    // TMP_100_SetLowLimit(&h_tmp_100_b);
    // TMP_100_SetConfRegisters(&h_tmp_100_b);



	TCA9544A_SelectChannel(&tca, VIPER_CARD_0);

}


void viper_superLoop(void){
	while (1) {
#ifdef VIPER_TEST
        int test_output_command = -1;
        uint8_t test_output_card = 0;
        uint32_t test_result = 0;
#endif

		VIPER_Card_Check(&viper_state);

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
				// global disable sets the inactive state
				VIPER_TEST_REPORT(DISABLE_MESSAGE | 0x80, 0, viper_state.STATE);
				break;
			}
			case ENABLE_MESSAGE: {
				viper_state.STATE = VIPER_STATE_ACTIVE;
				// global enable sets the active state
				VIPER_TEST_REPORT(ENABLE_MESSAGE | 0x80, 0, viper_state.STATE);
				break;
			}
			case HEALTH_STATUS_PING: {
				MX_CAN_Broadcast_Health_Message(&viper_can, &viper_state);
				// health ping was handled; report the current state
				VIPER_TEST_REPORT(HEALTH_STATUS_PING | 0x80, 0, viper_state.STATE);
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
				// check this card's enable pin after the GPIO update
				TEST_OUTPUT(DISABLE_CARD, new_message->card_id);
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
				// check all enable pins are low after the GPIO update
				TEST_OUTPUT(DISABLE_ALL_CARDS, 0);
				viper_state.CARD_0.ENABLE = 0;
				viper_state.CARD_1.ENABLE = 0;
				viper_state.CARD_2.ENABLE = 0;
				viper_state.CARD_3.ENABLE = 0;
				break;
			}
			case ENABLE_CARD: {
				// check this card's enable pin after the GPIO update
				TEST_OUTPUT(ENABLE_CARD, new_message->card_id);
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
				// check all enable pins are high after the GPIO update
				TEST_OUTPUT(ENABLE_ALL_CARDS, 0);
				viper_state.CARD_0.ENABLE = 1;
				viper_state.CARD_1.ENABLE = 1;
				viper_state.CARD_2.ENABLE = 1;
				viper_state.CARD_3.ENABLE = 1;
				break;
			}
			case GET_CARD_DATA: {
				MX_CAN_Broadcast_Card_Data(&viper_can, &viper_state, new_message->card_id);
				// the selected card's data-request handler completed
				VIPER_TEST_REPORT(GET_CARD_DATA, new_message->card_id, new_message->card_id);
				break;
			}
			case GET_ALL_CARD_DATA: {
				MX_CAN_Broadcast_Card_Data(&viper_can, &viper_state, VIPER_CARD_0);
				MX_CAN_Broadcast_Card_Data(&viper_can, &viper_state, VIPER_CARD_1);
				MX_CAN_Broadcast_Card_Data(&viper_can, &viper_state, VIPER_CARD_2);
				MX_CAN_Broadcast_Card_Data(&viper_can, &viper_state, VIPER_CARD_3);
				// the handler called broadcasts for all four cards
				VIPER_TEST_REPORT(GET_ALL_CARD_DATA, 0, 4);
				break;
			}
			case SET_MUX_VALUE: {
				// capture the mux driver's return status
				TEST_RESULT(TCA9544A_SelectChannel(&tca, new_message->card_id));
				viper_state.CURRENT_CARD = new_message->card_id;
				// report mux status and the selected channel
				VIPER_TEST_REPORT(SET_MUX_VALUE, new_message->card_id,
                    (test_result << 8) | tca.current_channel);
				break;
			}
			case SAVE_TO_EEPROM: {
				// capture the EEPROM write's return status
				TEST_RESULT(AT24C04C_WritePages(&at24c04c_1, (uint8_t*) &viper_params, sizeof(VIPER_PARAMS_TypeDef), VIPER_PARAMS_EEPROM_PAGE));
				// EEPROM write returned OK (0), not reboot persistence
				VIPER_TEST_REPORT(SAVE_TO_EEPROM, 0, test_result);
				break;
			}
			case SET_FREEZE: {
				FREEZE = 1;
				// freeze command sets FREEZE to 1
				VIPER_TEST_REPORT(SET_FREEZE, 0, FREEZE);
				break;
			}
			case STOP_FREEZE: {
				FREEZE = 0;
				// unfreeze command clears FREEZE to 0
				VIPER_TEST_REPORT(STOP_FREEZE, 0, FREEZE);
				break;
			}
			// GETTERS / SETTERS
			case GET_HEALTH_INTERVAL: {
				MX_CAN_Broadcast_Uint32_Data(&viper_can, viper_params.HEALTH_INTERVAL, GET_HEALTH_INTERVAL, 0);
				// report the current health interval
				VIPER_TEST_REPORT(GET_HEALTH_INTERVAL, 0, viper_params.HEALTH_INTERVAL);
				break;
			}
			case SET_HEALTH_INTERVAL: {
				viper_params.HEALTH_INTERVAL = decode_uint32_big_endian(new_message->data);
				// report the health interval stored by this command
				VIPER_TEST_REPORT(SET_HEALTH_INTERVAL, 0, viper_params.HEALTH_INTERVAL);
				break;
			}
			case GET_CARD_INTERVAL: {
				MX_CAN_Broadcast_Uint32_Data(&viper_can, viper_params.CARD_INTERVAL, GET_CARD_INTERVAL, 0);
				// report the current card interval
				VIPER_TEST_REPORT(GET_CARD_INTERVAL, 0, viper_params.CARD_INTERVAL);
				break;
			}
			case SET_CARD_INTERVAL: {
				viper_params.CARD_INTERVAL = decode_uint32_big_endian(new_message->data);
				// report the card interval stored by this command
				VIPER_TEST_REPORT(SET_CARD_INTERVAL, 0, viper_params.CARD_INTERVAL);
				break;
			}
#ifdef VIPER_TEST
            case 0x70:
                // save original settings so Python can restore them
                VIPER_TEST_REPORT(0x70, 0, test_output_mask()
                    | (viper_state.STATE << 8) | (FREEZE << 9) | (viper_state.CURRENT_CARD << 10));
                break;
#endif
			default:
				break;
			}
			free(new_message->data);
			queue_dequeue(&can_message_queue_viper);
		}

		 VIPER_Card_Update_State(&viper_state);
#ifdef VIPER_TEST
        // return enable-pin levels after applying the command
        if (test_output_command >= 0)
            VIPER_TEST_REPORT(test_output_command, test_output_card, test_output_mask());
#endif
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


			
			if ((viper_params.CARD_INTERVAL != 0) && (HAL_GetTick() % viper_params.CARD_INTERVAL == 0)) {

				if (!FREEZE) {
					if (viper_state.CURRENT_CARD == VIPER_CARD_3) {
						viper_state.CURRENT_CARD = VIPER_CARD_0;
					} else {
						viper_state.CURRENT_CARD++;
					}
				}

				read_complete = 0;
				send_complete = 0;
			}
		
			if (!read_complete)
			{
				if((viper_state.CURRENT_CARD == VIPER_CARD_0 && viper_state.CARD_0.ENABLE)
				|| (viper_state.CURRENT_CARD == VIPER_CARD_1 && viper_state.CARD_1.ENABLE)
				|| (viper_state.CURRENT_CARD == VIPER_CARD_2 && viper_state.CARD_2.ENABLE)
				|| (viper_state.CURRENT_CARD == VIPER_CARD_3 && viper_state.CARD_3.ENABLE))
				{
					VIPER_Card_Read(&viper_state, viper_state.CURRENT_CARD);
				}
				read_complete = 1;
			}
			break;
		}
		default:
			break;
		}

		// SCHEDULED TASKS
		// if ((viper_params.CARD_INTERVAL != 0) && (HAL_GetTick() % viper_params.CARD_INTERVAL == 0)) {
		// 	MX_CAN_Broadcast_Card_Data(&viper_can, &viper_state, viper_state.CURRENT_CARD);
		// }

		if (read_complete && !send_complete)
		{
			MX_CAN_Broadcast_Card_Data(&viper_can, &viper_state, viper_state.CURRENT_CARD);
			send_complete = 1;
		}

		//Poll and Publish CAN



#ifdef VIPER_TEST
        VIPER_TestPoll();
#endif
		MX_CAN_PollToEmptyQueue();



//		 if ((viper_params.HEALTH_INTERVAL != 0)
//		 		&& (HAL_GetTick() % viper_params.HEALTH_INTERVAL == 0)) {
//		 	MX_CAN_Broadcast_Health_Message(&viper_can, &viper_state);
//		 }

		// MAIN STATE MACHINE END

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	}
}


/* USER CODE BEGIN 4 */

void VIPER_Card_Init(VIPER_STATE_TypeDef *viper_state, VIPER_PARAMS_TypeDef *viper_params) {
	// Step 1: initialize viper_params (from EEPROM)
	AT24C04C_ReadPages(&at24c04c_1, (uint8_t*) viper_params, sizeof(VIPER_PARAMS_TypeDef), VIPER_PARAMS_EEPROM_PAGE);

	// Step 2: set any relevant fields in viper_state from viper_params
//	viper_state->CARD_0.ENABLE = viper_params->CARD_0.ENABLE;
	viper_state->CARD_0.ENABLE = 1; //Card zero is the jetson - ignore any eeprom for it, as it must always be on

	viper_state->CARD_1.ENABLE = viper_params->CARD_1.ENABLE;
	viper_state->CARD_2.ENABLE = viper_params->CARD_2.ENABLE;
	viper_state->CARD_3.ENABLE = viper_params->CARD_3.ENABLE;

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

	if (c0) {
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

	if (c1) {
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

	if (c2) {
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

	if (c3) {
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
		AT24C04C_WritePages(&at24c04c_1, (uint8_t*) viper_params, sizeof(VIPER_PARAMS_TypeDef), VIPER_PARAMS_EEPROM_PAGE);
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

	TMP_1075_ReadTemp(&h_tmp_1075);
	TMP_100_ReadTemp(&h_tmp_100_a);

	if (cardx != VIPER_CARD_2)
	{
		TMP_100_ReadTemp(&h_tmp_100_b);
	}
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

		MCP3221_ReadCurrent(&input_current_low_card);
	} else {

		INA_238_ReadCurrent(&ina238_high_power);
		INA_238_ReadVoltage(&ina238_high_power);
		//INA_238_ReadPower(&ina238_high_power);
		// INA_238_ReadDiagnostic(&ina238_high_power);

		MCP3221_ReadCurrent(&input_current_high_card);
	}

	//MCP3221_ReadVoltage(&input_voltage);

	// Step 3: Writing everything

	switch (cardx) {
		case VIPER_CARD_0: {

			viper_state->CARD_0.TEMPERATURE_BACKPLANE = h_tmp_1075.temp;
			viper_state->CARD_0.TEMPERATURE_CARD_A = h_tmp_100_a.temp;
			viper_state->CARD_0.TEMPERATURE_CARD_B = h_tmp_100_b.temp;
			viper_state->CARD_0.INPUT_CURRENT = input_current_low_card.current/1000;
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

			viper_state->CARD_1.TEMPERATURE_BACKPLANE = h_tmp_1075.temp;
			viper_state->CARD_1.TEMPERATURE_CARD_A = h_tmp_100_a.temp;
			viper_state->CARD_1.TEMPERATURE_CARD_B = h_tmp_100_b.temp;
			viper_state->CARD_1.INPUT_CURRENT = input_current_high_card.current/1000;
			viper_state->CARD_1.OUTPUT_CURRENT_A = ina238_high_power.current;
			viper_state->CARD_1.OUTPUT_VOLTAGE_A = ina238_high_power.voltage;
			// viper_state->CARD_1.OUTPUT_POWER_A = ina238_high_power.power;
			// viper_state->CARD_1.OUTPUT_DIAGNOSTIC_A = ina238_high_power.diagnostic;
			break;

		}
		case VIPER_CARD_2: {

			viper_state->CARD_2.TEMPERATURE_BACKPLANE = h_tmp_1075.temp;
			viper_state->CARD_2.TEMPERATURE_CARD_A = h_tmp_100_a.temp;
			viper_state->CARD_2.TEMPERATURE_CARD_B = h_tmp_100_b.temp;
			viper_state->CARD_2.INPUT_CURRENT = input_current_high_card.current/1000;
			viper_state->CARD_2.OUTPUT_CURRENT_A = ina238_high_power.current;
			viper_state->CARD_2.OUTPUT_VOLTAGE_A = ina238_high_power.voltage;
			// viper_state->CARD_2.OUTPUT_POWER_A = ina238_high_power.power;
			// viper_state->CARD_2.OUTPUT_DIAGNOSTIC_A = ina238_high_power.diagnostic;
			break;
		}

		case VIPER_CARD_3:  {

			 viper_state->CARD_3.TEMPERATURE_BACKPLANE = h_tmp_1075.temp;
			viper_state->CARD_3.TEMPERATURE_CARD_A = h_tmp_100_a.temp;
			viper_state->CARD_3.TEMPERATURE_CARD_B = h_tmp_100_b.temp;
			 viper_state->CARD_3.INPUT_CURRENT = input_current_low_card.current/1000;
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