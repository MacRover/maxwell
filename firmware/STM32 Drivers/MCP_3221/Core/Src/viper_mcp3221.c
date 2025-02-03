/*
 * mcp_3221.c
 *
 *  Created on: Nov 19, 2024
 *      Author: Ali Naqvi
 */

#include "viper_mcp3221.h"
#include "i2c.h"

MCP_3221_HandleTypeDef input_current_low_card;
MCP_3221_HandleTypeDef input_current_high_card;

MCP_3221_HandleTypeDef input_voltage;

void MX_MCP_3221_Init() {
    
//For low input current drivers 
    input_current_low_card.Init.hi2c = &hi2c2;
    input_current_low_card.Init.address_pins = VIPER_ADC_ADDRESS & 0x07;
    input_current_low_card.Init.vref_mv = VIPER_VREF_MV;
    input_current_low_card.Init.sense_resistor_ohms = VIPER_LOW_SENSE_RES; 
    input_current_low_card.Init.scaling_factor = VIPER_CURRENT_SCALING_FACTOR;
// -------------------------------
//For high input current drivers 
    input_current_high_card.Init.hi2c = &hi2c2;
    input_current_high_card.Init.address_pins = VIPER_ADC_ADDRESS & 0x07;
    input_current_high_card.Init.vref_mv = VIPER_VREF_MV;
    input_current_high_card.Init.sense_resistor_ohms = VIPER_HIGH_SENSE_RES; 
    input_current_high_card.Init.scaling_factor = VIPER_CURRENT_SCALING_FACTOR;

//for input voltage
    input_voltage.Init.hi2c = &hi2c1;
    input_voltage.Init.address_pins = VIPER_ADC_ADDRESS & 0x07;
    input_voltage.Init.vref_mv = VIPER_VREF_MV;
    input_voltage.Init.sense_resistor_ohms = VIPER_LOW_SENSE_RES;
    input_voltage.Init.scaling_factor = VIPER_VOLTAGE_SCALING_FACTOR; 


    if (MCP3221_Init(&input_current_low_card) != MCP_3221_OK)
    {
        Error_Handler();
    }

    if (MCP3221_Init(&input_current_high_card) != MCP_3221_OK)
    {
        Error_Handler();
    }

//    if (MCP3221_Init(&input_voltage) != MCP_3221_OK)
//    {
//        Error_Handler();
//    }
}
