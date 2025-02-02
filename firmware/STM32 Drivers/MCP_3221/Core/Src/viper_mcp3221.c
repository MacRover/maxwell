/*
 * mcp_3221.c
 *
 *  Created on: Nov 19, 2024
 *      Author: Ali Naqvi
 */

#include "viper_mcp3221.h"
#include "i2c.h"

// low power
MCP3221_HandleTypeDef mcp3221_card0_a;
MCP3221_HandleTypeDef mcp3221_card0_b;

//high power
MCP3221_HandleTypeDef mcp3221_card2;

void MX_MCP_3221_Init() {
    
    // Card A (low power)
    mcp3221_card0_a.Init.hi2c = &hi2c2;
    mcp3221_card0_a.Init.address_pins = VIPER_ADC_ADDRESS & 0x07;
    mcp3221_card0_a.Init.vref_mv = VIPER_VREF_MV;
    mcp3221_card0_a.Init.sense_resistor_ohms = VIPER_LOW_SENSE_RES;
    mcp3221_card0_a.Init.scaling_factor = VIPER_SCALING_FACTOR;

    // Card B (low power)
    mcp3221_card0_b.Init = mcp3221_card0_a.Init;

    // High power card
    mcp3221_card2.Init = mcp3221_card0_a.Init;
    mcp3221_card2.Init.sense_resistor_ohms = VIPER_HIGH_SENSE_RES;

    if (MCP3221_Init(&mcp3221_card0_a) != MCP_3221_OK)
    {
        Error_Handler();
    }

    if (MCP3221_Init(&mcp3221_card0_b) != MCP_3221_OK)
    {
        Error_Handler();
    }

    if (MCP3221_Init(&mcp3221_card2) != MCP_3221_OK)
    {
        Error_Handler();
    }
}
