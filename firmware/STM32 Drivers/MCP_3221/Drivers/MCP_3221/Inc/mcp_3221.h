/*
 * mcp_3221.h
 *
 *  Created on: Nov 19, 2024
 *      Author: Ali Naqvi
 */

#ifndef MCP3221_I2C_DRIVER_H
#define MCP3221_I2C_DRIVER_H

#include "stm32f1xx_hal.h"

#define MCP3221_RESOLUTION 4095
#define MCP3221_DATA_BYTES 2
#define MCP3221_FIXED_ID   0x09 // Fixed 4 bits from datasheet

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address_pins; // 3-bit configurable address
    float vref_mv;        // Reference voltage in millivolts
} MCP3221_Init_Struct;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t i2c_address;  // Full 7-bit address
    float vref_mv;
} MCP3221;

HAL_StatusTypeDef MCP3221_Init(MCP3221 *dev, MCP3221_Init_Struct *init_struct);

uint16_t MCP3221_readADC(MCP3221 *dev);

float MCP3221_getADCVoltage(MCP3221 *dev, uint16_t adc_value);

#endif

