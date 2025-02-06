/*
 * mcp_3221.c
 *
 *  Created on: Nov 19, 2024
 *      Author: Ali Naqvi
 */

// VIPER specific instantiation

#ifndef VIPER_MCP3221_H
#define VIPER_MCP3221_H

#include "mcp_3221.h"

extern MCP_3221_HandleTypeDef input_current_low_card;
extern MCP_3221_HandleTypeDef input_current_high_card;

extern MCP_3221_HandleTypeDef input_voltage;



// Define VIPER-specific constants
#define VIPER_VOLTAGE_SCALING_FACTOR        (17.6)      // Voltage divider scaling factor
#define VIPER_CURRENT_SCALING_FACTOR        (0.025)      // Voltage divider scaling factor
#define VIPER_LOW_SENSE_RES                 (0.002)     // Sense resistor value in ohms for low power card
#define VIPER_HIGH_SENSE_RES                (0.001)     // Sense resistor value in ohms for high power card
#define VIPER_VREF_MV                       (3300.0)    // Reference voltage in millivolts
#define VIPER_ADC_ADDRESS                     0x4D    	// I2C address for the MCP3221 device

// Function declarations
void MX_MCP_3221_Init();

#endif
