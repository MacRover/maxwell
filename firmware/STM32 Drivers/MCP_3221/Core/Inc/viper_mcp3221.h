// VIPER specific instantiation

#ifndef VIPER_MCP3221_H
#define VIPER_MCP3221_H

#include "mcp_3221.h"

// Define VIPER-specific constants
#define VIPER_SCALING_FACTOR float(17.6)    // Voltage divider scaling factor
#define VIPER_SENSE_RES       float(0.1)    // Sense resistor value in ohms
#define VIPER_VREF_MV       float(3300.0)   // Reference voltage in millivolts
#define VIPER_ADC_ADDRESS    0x4D    		// I2C address for the MCP3221 device

// Function declarations
HAL_StatusTypeDef VIPER_MCP3221_Init(I2C_HandleTypeDef *hi2c);

// Extra VIPER specific functions
float VIPER_GetInputVoltage(void);

float VIPER_GetInputCurrent(void);

#endif
