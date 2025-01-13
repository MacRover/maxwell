/*
 * ina238.c
 *
 *  Created on: Jan 10, 2025
 *      Author: ishan
 */

#include "stm32f1xx_ina238.h"

INA_238_StatusTypeDef INA_238_Init(INA_238_HandleTypeDef* ina_238)
{

}

INA_238_StatusTypeDef INA_238_DeInit(INA_238_HandleTypeDef* ina_238)
{

}

INA_238_StatusTypeDef INA_238_ReadCurrent(INA_238_HandleTypeDef* ina_238)
{

}

INA_238_StatusTypeDef INA_238_ReadVoltage(INA_238_HandleTypeDef* ina_238)
{

}

INA_238_StatusTypeDef INA_238_ReadPower(INA_238_HandleTypeDef* ina_238)
{

}

INA_238_StatusTypeDef INA_238_ReadDiagnostic(INA_238_HandleTypeDef* ina_238)
{

}

INA_238_StatusTypeDef INA_238_WriteConfig(INA_238_HandleTypeDef* ina_238)
{

}


//Below functions come from datasheet page 19
uint16_t __i2c_set_register_pointer(INA_238_HandleTypeDef* ina_238, uint8_t register_address)
{
    uint8_t i2c_address = ina_238->Init.device_identifier | (ina_238->Init.a1_pin << 3) | (ina_238->Init.a0_pin << 1) | 0; //a1 pin changes 3rd msb not 2nd msb

    while (HAL_I2C_Master_Transmit(ina_238->Init.I2C_HandlerInstance, (uint16_t)(i2c_address & 0xFE), &register_address, 1, 10000) != HAL_OK)
    {

        uint16_t I2C_Error = HAL_I2C_GetError(ina_238->Init.I2C_HandlerInstance);

        if (I2C_Error != HAL_I2C_ERROR_AF)
        {
            return I2C_Error;
        }
    }

    return HAL_I2C_ERROR_NONE;
}

uint16_t __i2c_read_register(INA_238_HandleTypeDef* ina_238, uint8_t *buffer)
{

    uint8_t i2c_address = ina_238->Init.device_identifier | (ina_238->Init.a1_pin << 3) | (ina_238->Init.a0_pin << 1) | 0; 

    while (HAL_I2C_Master_Receive(ina_238->Init.I2C_HandlerInstance, (uint16_t)(i2c_address & 0xFE), buffer, 2, 10000) != HAL_OK)
    {

        uint16_t I2C_Error = HAL_I2C_GetError(ina_238->Init.I2C_HandlerInstance);

        if (I2C_Error != HAL_I2C_ERROR_AF)
        {
            return I2C_Error;
        }
    }

    return HAL_I2C_ERROR_NONE;

}

uint16_t __i2c_write_register(INA_238_HandleTypeDef* ina_238, uint8_t register_address, uint8_t *data)
{

    uint8_t i2c_address = ina_238->Init.device_identifier | (ina_238->Init.a1_pin << 3) | (ina_238->Init.a0_pin << 1) | 0; 

    while (HAL_I2C_Mem_Write(ina_238->Init.I2C_HandlerInstance, (uint16_t)(i2c_address & 0xFE), (uint16_t) (register_address & 0xFF), I2C_MEMADD_SIZE_8BIT, data, 2, 10000) != HAL_OK)
    {
        uint16_t I2C_Error = HAL_I2C_GetError(ina_238->Init.I2C_HandlerInstance);

        if (I2C_Error != HAL_I2C_ERROR_AF)
        {
            return I2C_Error;
        }
    }

    return HAL_I2C_ERROR_NONE;

}