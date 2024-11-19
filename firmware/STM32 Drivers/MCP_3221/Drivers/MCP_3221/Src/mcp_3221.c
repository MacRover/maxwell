/*
 * mcp_3221.c
 *
 *  Created on: Nov 19, 2024
 *      Author: Ali Naqvi
 */

#include "mcp_3221.h"

HAL_StatusTypeDef MCP3221_Init(MCP3221 *dev, I2C_HandleTypeDef *hi2c, uint8_t i2c_Addr){

	dev->hi2c = hi2c;
    dev->i2c_Addr = i2c_Addr;

    // Built in error handling. Can be implemented as its own function...
    // Check communication with MCP3221 by attempting to read a dummy byte
    uint8_t dummy_data[MCP3221_DATA_BYTES] = {0};
    if (HAL_I2C_Master_Receive(hi2c, (uint16_t)(i2c_Addr << 1), dummy_data, MCP3221_DATA_BYTES, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

// TODO: last 4 functions...
