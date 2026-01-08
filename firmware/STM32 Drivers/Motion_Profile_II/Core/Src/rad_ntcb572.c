/*
 * rad_ntcb572.c
 *
 *  Created on: Mar 18, 2025
 *      Author: Ali
 */

#include "rad_ntcb572.h"

#include "adc.h"

// Global handle for the project-specific NTC driver
NTC_HandleTypeDef rad_ntc1;

void MX_RAD_NTCB572_Init(void)
{
    // 1) Assign the ADC handle used for reading the thermistor
    rad_ntc1.Init.hadc        = &hadc2;       // or whichever ADC is correct
    rad_ntc1.Init.Channel     = ADC_CHANNEL_8; // Example channel index

    // 2) Thermistor parameters
    rad_ntc1.Init.SupplyVoltage     = 3.3f;
    rad_ntc1.Init.RefResistorValue  = 10000.0f; // e.g., 10 kΩ
    rad_ntc1.Init.Bvalue            = 3455.0f;  // From datasheet (e.g. B25/85)
    rad_ntc1.Init.R25               = 10000.0f; // 10 kΩ at 25°C
    rad_ntc1.Init.T25               = 298.15f;  // 25°C in Kelvin
    rad_ntc1.Init.MaxAdcValue       = 4095;     // For 12-bit ADC

    // 3) Set initial driver state to RESET
    rad_ntc1.State     = NTC_STATE_RESET;
    rad_ntc1.ErrorCode = 0;

    // 4) Initialize the NTC driver
    if (NTC_Init(&rad_ntc1) != NTC_OK)
    {
        Error_Handler();
    }

    HAL_ADCEx_Calibration_Start(rad_ntc1.Init.hadc);
}
