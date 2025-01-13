/*
 * ina238.h
 *
 *  Created on: Jan 10, 2025
 *      Author: ishan
 */

#ifndef INA238_INC_STM32F1XX_INA238_H_
#define INA238_INC_STM32F1XX_INA238_H_

#include "stm32f1xx.h"

//Define register values

typedef enum
{
    INA_238_OK = 0x00U,
    INA_238_ERROR = 0x01U,
    INA_238_BUSY = 0x02U,
    INA_238_TIMEOUT = 0x03U
} INA_238_StatusTypeDef;


typedef struct
{
    uint16_t CONFIG;
    uint16_t ADC_CONFIG;
    uint16_t SHUNT_CAL;
    uint16_t DIAG_ALERT;
    uint16_t SOVL;
    uint16_t SUVL;
    uint16_t BOVL;
    uint16_t BUVL;
    uint16_t PWR_LIMIT;
} INA_238_ConfRegister_TypeDef;

typedef struct
{
	//Enter key params here

    I2C_HandleTypeDef *I2C_HandlerInstance; // i2c instance

    uint8_t a0_pin;

    uint8_t a1_pin;

    uint8_t device_identifier;

    float shunt_resistor;



} INA_238_InitTypeDef;

/**
 * @brief  INA_238 State structure definition
 */
typedef enum
{
    INA_238_STATE_RESET = 0x00U, /*!< Peripheral not Initialized                         */
    INA_238_STATE_READY = 0x01U, /*!< Peripheral Initialized and ready for use           */
    INA_238_STATE_BUSY = 0x02U, /*!< an internal process is ongoing                     */
    INA_238_STATE_ERROR = 0x06U, /*!< SPI error state                                    */
} INA_238_StateTypeDef;

typedef struct
{
    INA_238_InitTypeDef Init; 

    INA_238_ConfRegister_TypeDef ConfigurationRegisters;

    volatile INA_238_StateTypeDef state;

    //refer to section 8.2.2.5 of datasheet to see how these are converted
    float voltage;
    float current;
    float power;
    float diagnostic;
} INA_238_HandleTypeDef;



INA_238_StatusTypeDef INA_238_Init(INA_238_HandleTypeDef* ina_238);

INA_238_StatusTypeDef INA_238_DeInit(INA_238_HandleTypeDef* ina_238);

INA_238_StatusTypeDef INA_238_ReadCurrent(INA_238_HandleTypeDef* ina_238);
INA_238_StatusTypeDef INA_238_ReadVoltage(INA_238_HandleTypeDef* ina_238);
INA_238_StatusTypeDef INA_238_ReadPower(INA_238_HandleTypeDef* ina_238);
INA_238_StatusTypeDef INA_238_ReadDiagnostic(INA_238_HandleTypeDef* ina_238);
INA_238_StatusTypeDef INA_238_WriteConfig(INA_238_HandleTypeDef* ina_238);


uint16_t __i2c_set_register_pointer(INA_238_HandleTypeDef* ina_238, uint8_t register_address);

uint16_t __i2c_read_register(INA_238_HandleTypeDef* ina_238, uint8_t *buffer);

uint16_t __i2c_write_register(INA_238_HandleTypeDef* ina_238, uint8_t register_address, uint8_t *data);



#endif /* INA238_INC_INA238_H_ */
