/*
 * mcp_3221.c
 *
 *  Created on: Nov 19, 2024
 *      Author: Ali Naqvi
 */

#include "mcp_3221.h"

HAL_StatusTypeDef MCP3221_Init(MCP3221 *dev, I2C_HandleTypeDef *hi2c, uint8_t i2c_Addr){

	dev->hi2c = hi2c;
    dev->i2c_Addr = i2c_Addr;

    // Built in error handling. Can be implemented as its own function...
    // Check communication with MCP3221 by attempting to read a dummy byte
    uint8_t dummy_data[MCP3221_DATA_BYTES] = {0};
    if (HAL_I2C_Master_Receive(hi2c, (uint16_t)(i2c_Addr << 1), dummy_data, MCP3221_DATA_BYTES, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}


uint16_t MCP3221_readADC(MCP3221 *dev){
    uint8_t data[2];  // data [0] is the upper byte which will have its first 4 bits masked as per VIPER spec
    uint16_t adc_value = 0; // value to return (will be 12 bits)

    /*
     * I2C call: The second argument follows the project spec where the last address bit is set to 1 by first
     * left shifting the 7 bit i2c address. For example if i2c address is 8'b01001101 then the operation will
     * First left shift (8'10011010) and then or it with 0x01 making the LSB into 1 indicating a read operation.
    */
    if (HAL_I2C_Master_Receive(dev->hi2c, (dev->i2c_Addr << 1) | 1, data, 2, HAL_MAX_DELAY) != HAL_OK){
        return 0xFFFF; // error
    }
    /*
     * Mask off the upper 4 bits of the upper byte, data[0].
     * Then shift and concatenate with lower byte
    */
    adc_value = ((data[0] & 0x0F) << 8) | data[1];

    return adc_value;
}

float MCP3221_getADCVoltage(uint16_t adc_value){

	/*
	 * Voltage = ( ADC value / resolution ) * Reference(AKA input) Voltage
	*/

    float voltage = ((float)adc_value / MCP3221_RESOLUTION) * MCP3221_VREF_MV;
    return voltage;
}

float MCP3221_getInputVoltage(uint16_t adc_value){

	// Get voltage (V_ADC)
    float v_adc_mv = MCP3221_getADCVoltage(adc_value);

    // Convert mV to volts
    float v_adc_v = v_adc_mv / 1000.0;

    /*
     * Apply the resistor voltage divider scaling factor (p.22 project spec)
     * to calculate V_BAT.
    */
    float v_bat = v_adc_v * 17.6; //17.6 is the computed voltage divider scaling factor as per proj spec.

    return v_bat;
}

float MCP3221_getInputCurrent(uint16_t adc_value, float sense_res){

	// Get voltage (V_ADC)
    float v_adc_mv = MCP3221_getADCVoltage(adc_value);

    // Convert mV to volts
    float v_adc_v = v_adc_mv / 1000.0;

    float v_sns = v_adc_v / 40.0; // divisor is 40 according to the project spec (p.25)

    float input_current = v_sns / sense_res; // sense res will be different based on high/low power card

    return input_current;
}
