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

typedef enum
{
    MCP_3221_OK = 0x00U,
    MCP_3221_ERROR = 0x01U,
    MCP_3221_BUSY = 0x02U,
    MCP_3221_TIMEOUT = 0x03U
} MCP_3221_StatusTypeDef;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address_pins; // 3-bit configurable address
    float vref_mv;        // Reference voltage in millivolts
} MCP_3221_InitTypeDef;

typedef struct
{
    MCP_3221_InitTypeDef Init;  // Nested init struct
    uint8_t i2c_address;  // Full 7-bit I2C address (stored separately)
    double voltage;  // Last measured voltage
    double current;  // Last measured current
} MCP3221_HandleTypeDef;

/* Public API */
MCP3221_StatusTypeDef MCP3221_Init(MCP3221_HandleTypeDef *mcp3221);
MCP3221_StatusTypeDef MCP3221_ReadVoltage(MCP3221_HandleTypeDef *mcp3221);
MCP3221_StatusTypeDef MCP3221_ReadCurrent(MCP3221_HandleTypeDef *mcp3221);

/* Private function */
uint16_t __i2c_read_register(INA_238_HandleTypeDef *mcp3221);

#endif

