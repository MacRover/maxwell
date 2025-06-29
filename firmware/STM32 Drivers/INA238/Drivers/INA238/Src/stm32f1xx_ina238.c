/*
 * ina238.c
 *
 *  Created on: Jan 10, 2025
 *      Author: ishan
 */

#include "stm32f1xx_ina238.h"

INA_238_StatusTypeDef INA_238_Init(INA_238_HandleTypeDef *ina_238)
{
    if (ina_238 == NULL)
        return INA_238_ERROR;
    if (ina_238->state == INA_238_STATE_READY)
        return INA_238_ERROR;
    if (ina_238->state == INA_238_STATE_BUSY)
        return INA_238_BUSY;
    if (ina_238->state == INA_238_STATE_ERROR)
        return INA_238_ERROR;
    if (ina_238->Init.I2C_HandlerInstance == NULL)
        return INA_238_ERROR;
    ina_238->state = INA_238_STATE_READY;
    return INA_238_OK;
}

INA_238_StatusTypeDef INA_238_DeInit(INA_238_HandleTypeDef *ina_238)
{
    if (ina_238 == NULL)
        return INA_238_ERROR;
    if (ina_238->state == INA_238_STATE_RESET)
        return INA_238_ERROR;
    if (ina_238->state == INA_238_STATE_BUSY)
        return INA_238_BUSY;
    if (ina_238->state == INA_238_STATE_ERROR)
        return INA_238_ERROR;
    ina_238->state = INA_238_STATE_RESET;
    return INA_238_OK;
}

INA_238_StatusTypeDef INA_238_ReadCurrent(INA_238_HandleTypeDef *ina_238)
{
    if (__i2c_set_register_pointer(ina_238, INA238_CURRENT_ADDRESS) != HAL_I2C_ERROR_NONE)
        return INA_238_ERROR;
    int16_t current_reading;
    if (__i2c_read_register_2Bytes(ina_238, (uint8_t *)&current_reading) != HAL_I2C_ERROR_NONE)
        return INA_238_ERROR;
    ina_238->current = current_reading * (ina_238->Init.max_expected_current / 32768.0);
    return INA_238_OK;
}

INA_238_StatusTypeDef INA_238_ReadVoltage(INA_238_HandleTypeDef *ina_238)
{
    if (__i2c_set_register_pointer(ina_238, INA238_VBUS_ADDRESS) != HAL_I2C_ERROR_NONE)
        return INA_238_ERROR;
    int16_t voltage_reading;
    if (__i2c_read_register_2Bytes(ina_238, (uint8_t *)&voltage_reading) != HAL_I2C_ERROR_NONE)
        return INA_238_ERROR;
    ina_238->voltage = voltage_reading * 0.003125;
    return INA_238_OK;
}

INA_238_StatusTypeDef INA_238_ReadPower(INA_238_HandleTypeDef *ina_238)
{
    if (__i2c_set_register_pointer(ina_238, INA238_POWER_ADDRESS) != HAL_I2C_ERROR_NONE)
        return INA_238_ERROR;
    int32_t power_reading;
    if (__i2c_read_register_3Bytes(ina_238, (uint8_t *)&power_reading) != HAL_I2C_ERROR_NONE)
        return INA_238_ERROR;

    ina_238->power = 0.2 * power_reading * (ina_238->Init.max_expected_current / 32768.0);
    return INA_238_OK;
}

INA_238_StatusTypeDef INA_238_ReadDiagnostic(INA_238_HandleTypeDef *ina_238)
{
    return INA_238_OK;
}

INA_238_StatusTypeDef INA_238_WriteConfig(INA_238_HandleTypeDef *ina_238)
{
    if (__i2c_write_register(ina_238, INA238_CONFIG_ADDRESS, ina_238->ConfigurationRegisters.CONFIG) != HAL_I2C_ERROR_NONE)
        return INA_238_ERROR;
    if (__i2c_write_register(ina_238, INA238_ADC_CONFIG_ADDRESS, ina_238->ConfigurationRegisters.ADC_CONFIG) != HAL_I2C_ERROR_NONE)
        return INA_238_ERROR;
    if (__i2c_write_register(ina_238, INA238_SHUNT_CAL_ADDRESS, ina_238->ConfigurationRegisters.SHUNT_CAL) != HAL_I2C_ERROR_NONE)
        return INA_238_ERROR;
    if (__i2c_write_register(ina_238, INA238_DIAG_ALRT_ADDRESS, ina_238->ConfigurationRegisters.DIAG_ALERT) != HAL_I2C_ERROR_NONE)
        return INA_238_ERROR;
    if (__i2c_write_register(ina_238, INA238_SOVL_ADDRESS, ina_238->ConfigurationRegisters.SOVL) != HAL_I2C_ERROR_NONE)
        return INA_238_ERROR;
    if (__i2c_write_register(ina_238, INA238_SUVL_ADDRESS, ina_238->ConfigurationRegisters.SUVL) != HAL_I2C_ERROR_NONE)
        return INA_238_ERROR;
    if (__i2c_write_register(ina_238, INA238_BOVL_ADDRESS, ina_238->ConfigurationRegisters.BOVL) != HAL_I2C_ERROR_NONE)
        return INA_238_ERROR;
    if (__i2c_write_register(ina_238, INA238_BUVL_ADDRESS, ina_238->ConfigurationRegisters.BUVL) != HAL_I2C_ERROR_NONE)
        return INA_238_ERROR;
    if (__i2c_write_register(ina_238, INA238_PWR_LIMIT_ADDRESS, ina_238->ConfigurationRegisters.PWR_LIMIT) != HAL_I2C_ERROR_NONE)
        return INA_238_ERROR;
    return INA_238_OK;
}

// Below functions come from datasheet page 19
uint16_t __i2c_set_register_pointer(INA_238_HandleTypeDef *ina_238, uint8_t register_address)
{
    uint8_t i2c_address = ina_238->Init.device_identifier | (ina_238->Init.a1_pin << 3) | (ina_238->Init.a0_pin << 1) | 0; // a1 pin changes 3rd msb not 2nd msb

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

uint16_t __i2c_read_register_2Bytes(INA_238_HandleTypeDef *ina_238, uint8_t *buffer)
{
    uint8_t spi_read[2];
    uint8_t i2c_address = ina_238->Init.device_identifier | (ina_238->Init.a1_pin << 3) | (ina_238->Init.a0_pin << 1) | 0;

    while (HAL_I2C_Master_Receive(ina_238->Init.I2C_HandlerInstance, (uint16_t)(i2c_address & 0xFE), spi_read, 2, 10000) != HAL_OK)
    {

        uint16_t I2C_Error = HAL_I2C_GetError(ina_238->Init.I2C_HandlerInstance);

        if (I2C_Error != HAL_I2C_ERROR_AF)
        {
            return I2C_Error;
        }
    }

    buffer[0] = spi_read[1];
    buffer[1] = spi_read[0];

    return HAL_I2C_ERROR_NONE;
}

uint16_t __i2c_read_register_3Bytes(INA_238_HandleTypeDef *ina_238, uint8_t *buffer)
{
    uint8_t spi_read[3];
    uint8_t i2c_address = ina_238->Init.device_identifier | (ina_238->Init.a1_pin << 3) | (ina_238->Init.a0_pin << 1) | 0;

    while (HAL_I2C_Master_Receive(ina_238->Init.I2C_HandlerInstance, (uint16_t)(i2c_address & 0xFE), spi_read, 3, 10000) != HAL_OK)
    {

        uint16_t I2C_Error = HAL_I2C_GetError(ina_238->Init.I2C_HandlerInstance);

        if (I2C_Error != HAL_I2C_ERROR_AF)
        {
            return I2C_Error;
        }
    }

    buffer[0] = spi_read[2];
    buffer[1] = spi_read[1];
    buffer[2] = spi_read[0];

    return HAL_I2C_ERROR_NONE;
}

uint16_t __i2c_write_register(INA_238_HandleTypeDef *ina_238, uint8_t register_address, uint16_t data)
{
    uint8_t buffer[] = {(uint8_t)((data & 0xFF00) >> 8), (uint8_t)(data & 0x00FF)};
    uint8_t i2c_address = ina_238->Init.device_identifier | (ina_238->Init.a1_pin << 3) | (ina_238->Init.a0_pin << 1) | 0;

    while (HAL_I2C_Mem_Write(ina_238->Init.I2C_HandlerInstance, (uint16_t)(i2c_address & 0xFE), (uint16_t)(register_address & 0xFF), I2C_MEMADD_SIZE_8BIT, buffer, 2, 10000) != HAL_OK)
    {
        uint16_t I2C_Error = HAL_I2C_GetError(ina_238->Init.I2C_HandlerInstance);

        if (I2C_Error != HAL_I2C_ERROR_AF)
        {
            return I2C_Error;
        }
    }

    return HAL_I2C_ERROR_NONE;
}
