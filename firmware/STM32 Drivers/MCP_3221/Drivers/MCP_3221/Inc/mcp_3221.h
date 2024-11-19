/*
 * mcp_3221.h
 *
 *  Created on: Nov 19, 2024
 *      Author: Ali Naqvi
 */

#ifndef MCP3221_I2C_DRIVER_H
#define MCP3221_I2C_DRIVER_H

#include "stm32f1xx_hal.h" 							// hardware abstraction for i2c

#define MCP3221_DEVICE_ID		0x4D 				// According to VIPER backplane schematic

#define MCP3221_RESOLUTION  	4095     			// 12-bit resolution
#define MCP3221_VREF_MV         3300				// in mV (is 3.3V according to VIPER backplane)
#define MCP3221_DATA_BYTES		2

// define constructor
typedef struct {
    I2C_HandleTypeDef *hi2c;  						// Pointer to the I2C handle
    uint8_t i2c_Addr; 								// I2C address of the MCP3221 (7-bit address)
} MCP3221;

HAL_StatusTypeDef MCP3221_Init(MCP3221 *dev, I2C_HandleTypeDef *hi2c, uint8_t i2c_Addr);

uint16_t MCP3221_readADC(MCP3221 *dev);

float MCP3221_getADCVoltage(uint16_t adc_value);

float MCP3221_getInputVoltage(uint16_t adc_value);

float MCP3221_getInputCurrent(uint16_t adc_value, float sense_res);

#endif

