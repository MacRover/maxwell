/*
 * mcp_3221.c
 *
 *  Created on: Nov 19, 2024
 *      Author: Ali Naqvi
 */

#include "mcp_3221.h"

HAL_StatusTypeDef MCP3221_Init(MCP3221 *dev, MCP3221_Init_Struct *init_struct) {
    dev->hi2c = init_struct->hi2c;
    dev->vref_mv = init_struct->vref_mv;
    dev->i2c_address = (MCP3221_FIXED_ID << 3) | (init_struct->address_pins & 0x07);

    uint8_t dummy_data[MCP3221_DATA_BYTES] = {0};
    if (HAL_I2C_Master_Receive(dev->hi2c, (dev->i2c_address << 1), dummy_data, MCP3221_DATA_BYTES, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

uint16_t MCP3221_readADC(MCP3221 *dev) {
    uint8_t data[MCP3221_DATA_BYTES];  // data [0] is the upper byte which will have its first 4 bits masked
    uint16_t adc_value = 0;

    if (HAL_I2C_Master_Receive(dev->hi2c, (dev->i2c_address << 1) | 1, data, MCP3221_DATA_BYTES, HAL_MAX_DELAY) != HAL_OK) {
        return 0xFFFF;
    }

    adc_value = ((data[0] & 0x0F) << 8) | data[1];

    return adc_value;
}

float MCP3221_getADCVoltage(MCP3221 *dev, uint16_t adc_value) {
	float voltage = ((float)adc_value / MCP3221_RESOLUTION) * dev->vref_mv;
    return voltage;
}


