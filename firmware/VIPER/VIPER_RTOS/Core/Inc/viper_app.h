/*
 * viper_app.h
 *
 *  Created on: Oct 23, 2026
 *      Author: ferrm
 */

#ifndef INC_VIPER_APP_H_
#define INC_VIPER_APP_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum
{
	VIPER_STATE_INACTIVE,
	VIPER_STATE_ACTIVE
} VIPER_STATE_MACHINE_TypeDef;

typedef enum
{
	VIPER_CARD_0,
	VIPER_CARD_1,
	VIPER_CARD_2,
	VIPER_CARD_3
} VIPER_CARD_ID_TypeDef;

typedef enum
{
	VIPER_CARD_OK,
	VIPER_CARD_FAULT,
	VIPER_CARD_DISCONNECTED
} VIPER_CARD_STATUS_TypeDef;

typedef struct __attribute__((packed))
{
	uint8_t ENABLE;
	double MAX_INPUT_CURRENT;
	double MAX_OUTPUT_CURRENT_A;
	double MAX_OUTPUT_CURRENT_B;
	double MAX_OUTPUT_VOLTAGE_A;
	double MAX_OUTPUT_VOLTAGE_B;
	double MAX_OUTPUT_POWER_A;
	double MAX_OUTPUT_POWER_B;
	double RESISTANCE;
} VIPER_CARD_PARAMS_TypeDef;

typedef struct
{
	uint8_t ENABLE;
	uint8_t CONNECTED;
	uint8_t INPUT_FAULT;
	uint8_t OUTPUT_FAULT_A;
	uint8_t OUTPUT_FAULT_B;
	VIPER_CARD_STATUS_TypeDef STATUS;
	double INPUT_CURRENT;
	double OUTPUT_CURRENT_A;
	double OUTPUT_CURRENT_B;
	double OUTPUT_VOLTAGE_A;
	double OUTPUT_VOLTAGE_B;
	double OUTPUT_POWER_A; // todo: incorporate into CAN call
	double OUTPUT_POWER_B; // todo: incorporate into CAN call
	double TEMPERATURE_BACKPLANE;
	double TEMPERATURE_CARD_A;
	double TEMPERATURE_CARD_B;
	double OUTPUT_DIAGNOSTIC_A;
	double OUTPUT_DIAGNOSTIC_B;
} VIPER_CARD_STATE_TypeDef;

typedef struct __attribute__((packed))
{
	uint8_t VIPER_ID;
 	uint32_t HEALTH_INTERVAL;
 	uint32_t CARD_INTERVAL;
 	VIPER_CARD_PARAMS_TypeDef CARD_0;
 	VIPER_CARD_PARAMS_TypeDef CARD_1;
 	VIPER_CARD_PARAMS_TypeDef CARD_2;
 	VIPER_CARD_PARAMS_TypeDef CARD_3;
} VIPER_PARAMS_TypeDef;

typedef struct
{
	VIPER_STATE_MACHINE_TypeDef STATE;
	VIPER_CARD_ID_TypeDef CURRENT_CARD;
	VIPER_CARD_STATE_TypeDef CARD_0;
	VIPER_CARD_STATE_TypeDef CARD_1;
	VIPER_CARD_STATE_TypeDef CARD_2;
	VIPER_CARD_STATE_TypeDef CARD_3;

	// UPDATE THIS TO INCLUDE ERRORS FROM DRIVERS
	// ENSURE EACH LIBRARY IS SENDING APPROPRIATE ERRORS
	uint8_t EEPROM_STATUS;
	uint8_t MUX_STATUS;
} VIPER_STATE_TypeDef;

extern VIPER_PARAMS_TypeDef viper_params; // static state values
extern VIPER_STATE_TypeDef viper_state; // dynamic state values
extern uint8_t ESTOP;
extern uint8_t FREEZE;
extern uint8_t read_complete;
extern uint8_t send_complete;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void viper_mainInit(void);
void viper_superLoop(void);
void VIPER_Card_Init(VIPER_STATE_TypeDef *viper_state, VIPER_PARAMS_TypeDef *viper_params);
void VIPER_Card_Check(VIPER_STATE_TypeDef* viper_state);
void VIPER_Card_Update_Params(VIPER_STATE_TypeDef* viper_state, VIPER_PARAMS_TypeDef *viper_params);
void VIPER_Card_Update_State(VIPER_STATE_TypeDef* viper_state);
void VIPER_Card_Read(VIPER_STATE_TypeDef* viper_state, VIPER_CARD_ID_TypeDef cardx);
uint8_t VIPER_Card_Params_Flag(VIPER_STATE_TypeDef* viper_state, VIPER_PARAMS_TypeDef *viper_params);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* INC_VIPER_APP_H_ */
