/*
 * ina238.c
 *
 *  Created on: Jan 10, 2025
 *      Author: ishan
 */



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


uint16_t __i2c_read(INA_238_HandleTypeDef* ina_238, uint16_t address, uint8_t *buffer, uint16_t len_bytes)
{

    uint8_t i2c_address = ina_238->Init.device_identifier | (ina_238->Init.a2_pin << 3) | (ina_238->Init.a1_pin << 2) | ((address & 0x1FF) >> 8) << 1 | 1; // limit eeprom address to 9 bits, get eeprom addr MSB, read command

    while (HAL_I2C_Mem_Read(ina_238->Init.I2C_HandlerInstance, (uint16_t)(i2c_address & 0xFE), (uint16_t) (address & 0xFF), I2C_MEMADD_SIZE_8BIT, buffer, len_bytes, 10000) != HAL_OK)
    {

        uint16_t I2C_Error = HAL_I2C_GetError(ina_238->Init.I2C_HandlerInstance);

        if (I2C_Error != HAL_I2C_ERROR_AF)
        {
            return I2C_Error;
        }
    }

    return HAL_I2C_ERROR_NONE;

}

uint16_t __i2c_write(INA_238_HandleTypeDef* ina_238, uint16_t address, uint8_t *data, uint16_t len_bytes)
{

    uint8_t i2c_address = ina_238->Init.device_identifier | (ina_238->Init.a2_pin << 3) | (ina_238->Init.a1_pin << 2) | ((address & 0x1FF) >> 8) << 1 | 0; // limit eeprom address to 9 bits, get eeprom addr MSB, write command

    while (HAL_I2C_Mem_Write(ina_238->Init.I2C_HandlerInstance, (uint16_t)(i2c_address & 0xFE), (uint16_t) (address & 0xFF), I2C_MEMADD_SIZE_8BIT, data, len_bytes, 10000) != HAL_OK)
    {
        uint16_t I2C_Error = HAL_I2C_GetError(ina_238->Init.I2C_HandlerInstance);

        if (I2C_Error != HAL_I2C_ERROR_AF)
        {
            return I2C_Error;
        }
    }

    return HAL_I2C_ERROR_NONE;

}