/*
 * mcp_3221.c
 *
 *  Created on: Nov 19, 2024
 *      Author: Ali Naqvi
 */

#include "mcp_3221.h"

// DONE, need to revise at the end. 
MCP3221_StatusTypeDef MCP3221_Init(MCP3221_HandleTypeDef *mcp3221) {

    if (mcp3221 == NULL)
        return MCP3221_ERROR;
    if (mcp3221->Init.I2C_HandlerInstance == NULL)
        return MCP3221_ERROR;

    mcp3221->i2c_address = (MCP3221_FIXED_ID << 3) | (mcp3221->Init.address_pins & 0x07);

    // Perform a dummy read to check communication
    if (__MCP3221_ReadRegister(mcp3221) != MCP3221_OK)
        return MCP3221_ERROR;

    return MCP3221_OK;
}

/*

-------------------- REFACTOR into private function -------------------------

uint16_t MCP3221_readADC(MCP3221 *dev) {
    uint8_t data[MCP3221_DATA_BYTES];  // data [0] is the upper byte which will have its first 4 bits masked
    uint16_t adc_value = 0;

    if (HAL_I2C_Master_Receive(dev->hi2c, (dev->i2c_address << 1) | 1, data, MCP3221_DATA_BYTES, HAL_MAX_DELAY) != HAL_OK) {
        return 0xFFFF;
    }

    adc_value = ((data[0] & 0x0F) << 8) | data[1];

    return adc_value;
}
*/

/*

------------------------- REFACTOR and make it utilize the private function above ------

float MCP3221_getADCVoltage(MCP3221 *dev, uint16_t adc_value) {
	float voltage = ((float)adc_value / MCP3221_RESOLUTION) * dev->vref_mv;
    return voltage;
}
*/


