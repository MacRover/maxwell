/*
 * mcp_3221.c
 *
 *  Created on: Nov 19, 2024
 *      Author: Ali Naqvi
 */

#include "mcp_3221.h"

// DONE, need to revise. 
MCP_3221_StatusTypeDef MCP3221_Init(MCP_3221_HandleTypeDef *mcp3221) {

    if (mcp3221 == NULL)
        return MCP_3221_ERROR;
    if (mcp3221->Init.hi2c == NULL)
        return MCP_3221_ERROR;

    mcp3221->i2c_address = (MCP3221_FIXED_ID << 3) | (mcp3221->Init.address_pins & 0x07);

    // // Perform a dummy read to check communication, (can also use private "__i2c_read_register" function)
    // uint8_t dummy_data[MCP3221_DATA_BYTES] = {0};
    // if (HAL_I2C_Master_Receive(mcp3221->Init.hi2c, (mcp3221->i2c_address << 1), dummy_data, MCP3221_DATA_BYTES, HAL_MAX_DELAY) != HAL_OK) {
    //     uint16_t error = HAL_I2C_GetError(mcp3221->Init.hi2c);
    //     return (error == HAL_I2C_ERROR_AF) ? MCP_3221_BUSY : MCP_3221_ERROR;
    // }

    return MCP_3221_OK;
}

MCP_3221_StatusTypeDef MCP3221_ReadVoltage(MCP_3221_HandleTypeDef *mcp3221) {
    if (__i2c_read_register(mcp3221) != MCP_3221_OK)
        return MCP_3221_ERROR;

    mcp3221->voltage = ((float)mcp3221->adc_value / MCP3221_RESOLUTION) * mcp3221->Init.vref_mv;
    mcp3221->voltage *= mcp3221->Init.scaling_factor;
    return MCP_3221_OK;
}

// DONE, need to revise. 
MCP_3221_StatusTypeDef MCP3221_ReadCurrent(MCP_3221_HandleTypeDef *mcp3221){

    if (__i2c_read_register(mcp3221) != MCP_3221_OK)
        return MCP_3221_ERROR;

    double temp_voltage;
    temp_voltage = ((float)mcp3221->adc_value / MCP3221_RESOLUTION) * mcp3221->Init.vref_mv;
    mcp3221->current = (temp_voltage*mcp3221->Init.scaling_factor) / mcp3221->Init.sense_resistor_ohms;
    return MCP_3221_OK;
}

// DONE, need to revise. 
MCP_3221_StatusTypeDef __i2c_read_register(MCP_3221_HandleTypeDef *mcp3221){
    uint8_t data[MCP3221_DATA_BYTES];  // data [0] is the upper byte which will have its first 4 bits masked
    if (HAL_I2C_Master_Receive(mcp3221->Init.hi2c, (mcp3221->i2c_address << 1) | 1, data, MCP3221_DATA_BYTES, HAL_MAX_DELAY) != HAL_OK) {
        uint16_t error = HAL_I2C_GetError(mcp3221->Init.hi2c);
        return (error == HAL_I2C_ERROR_AF) ? MCP_3221_BUSY : MCP_3221_ERROR;
    }
    mcp3221->adc_value = ((data[0] & 0x0F) << 8) | data[1];
    return MCP_3221_OK;
}


