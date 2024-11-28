#include "viper_mcp3221.h"

// Define project-specific parameters
#define VIPER_VREF_MV        (float)3300.0
#define VIPER_SCALING_FACTOR (float)17.6
#define VIPER_SENSE_RES      (float)0.1
#define VIPER_ADC_ADDRESS    0x4D

MCP3221 viper_adc;

HAL_StatusTypeDef VIPER_MCP3221_Init(I2C_HandleTypeDef *hi2c) {
    MCP3221_Init_Struct init_struct = {
        .hi2c = hi2c,
        .address_pins = VIPER_ADC_ADDRESS & 0x07, // Last 3 bits
        .vref_mv = VIPER_VREF_MV
    };
    return MCP3221_Init(&viper_adc, &init_struct);
}

float VIPER_GetInputVoltage() {
    uint16_t adc_value = MCP3221_readADC(&viper_adc); // get ADC value
    if (adc_value == 0xFFFF) return -1.0; // Error
    float v_adc = MCP3221_getADCVoltage(&viper_adc, adc_value);
    return (v_adc * VIPER_SCALING_FACTOR);
}

float VIPER_GetInputCurrent() {
    uint16_t adc_value = MCP3221_readADC(&viper_adc);
    if (adc_value == 0xFFFF) return -1.0; // Error
    float v_adc = MCP3221_getADCVoltage(&viper_adc, adc_value);
    return (v_adc / VIPER_SCALING_FACTOR) / VIPER_SENSE_RES;
}
