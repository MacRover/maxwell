/*
 * ntcb572.c
 *
 *  Created on: Mar 18, 2025
 *      Author: Ali
 */

#include <math.h>   // for logf()
#include "ntcb572.h"

NTC_StatusTypeDef NTC_Init(NTC_HandleTypeDef *hntc)
{
    // Check for NULL handle
    if (hntc == NULL)
    {
        return NTC_ERROR;
    }

    // Check current driver state
    if (hntc->State == NTC_STATE_READY)
    {
        // Already initialized
        return NTC_ERROR;
    }
    if (hntc->State == NTC_STATE_BUSY)
    {
        // Currently busy
        return NTC_BUSY;
    }
    if (hntc->State == NTC_STATE_ERROR)
    {
        // Error state
        return NTC_ERROR;
    }

    // Validate the init parameters
    if (hntc->Init.hadc == NULL)
    {
        // Must have a valid ADC handle to read thermistor voltage
        return NTC_ERROR;
    }
    if (hntc->Init.MaxAdcValue == 0)
    {
        // Prevent division by zero
        return NTC_ERROR;
    }

    // Transition to BUSY state during init
    hntc->State = NTC_STATE_BUSY;
    hntc->ErrorCode = 0x00U;

    // Initialize last known values
    hntc->lastResistance = 0.0f;
    hntc->lastTemperatureC = 25.0f;

    // Transition to READY state
    hntc->State = NTC_STATE_READY;

    return NTC_OK;
}

NTC_StatusTypeDef NTC_DeInit(NTC_HandleTypeDef *hntc)
{
    if (hntc == NULL)
    {
        return NTC_ERROR;
    }

    if (hntc->State == NTC_STATE_RESET)
    {
        // Already de-initialized
        return NTC_ERROR;
    }
    if (hntc->State == NTC_STATE_BUSY)
    {
        // Ongoing conversion or operation
        return NTC_BUSY;
    }
    if (hntc->State == NTC_STATE_ERROR)
    {
        return NTC_ERROR;
    }

    // Reset state
    hntc->State = NTC_STATE_RESET;
    hntc->ErrorCode = 0;

    return NTC_OK;
}

NTC_StatusTypeDef NTC_ReadRawAdc(NTC_HandleTypeDef *hntc, uint16_t *pRawAdcValue)
{
    if ((hntc == NULL) || (pRawAdcValue == NULL))
    {
        return NTC_ERROR;
    }

    if (hntc->State == NTC_STATE_RESET)
    {
        return NTC_ERROR;
    }
    if (hntc->State == NTC_STATE_BUSY)
    {
        return NTC_BUSY;
    }
    if (hntc->State == NTC_STATE_ERROR)
    {
        return NTC_ERROR;
    }

    hntc->State = NTC_STATE_BUSY;
    hntc->ErrorCode = 0;

    // Configure ADC channel
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = hntc->Init.Channel;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5; // Example
    if (HAL_ADC_ConfigChannel(hntc->Init.hadc, &sConfig) != HAL_OK)
    {
        hntc->State = NTC_STATE_ERROR;
        return NTC_ERROR;
    }

    // Start ADC, poll for conversion, retrieve value
    if (HAL_ADC_Start(hntc->Init.hadc) != HAL_OK)
    {
        hntc->State = NTC_STATE_ERROR;
        return NTC_ERROR;
    }
    if (HAL_ADC_PollForConversion(hntc->Init.hadc, 10) != HAL_OK)
    {
        // Timeout or other poll error
        HAL_ADC_Stop(hntc->Init.hadc);
        hntc->State = NTC_STATE_ERROR;
        return NTC_ERROR;
    }

    // Get the raw ADC reading
    *pRawAdcValue = (uint16_t)HAL_ADC_GetValue(hntc->Init.hadc);

    // Stop ADC (non continous sampling)
    HAL_ADC_Stop(hntc->Init.hadc);

    hntc->State = NTC_STATE_READY;

    return NTC_OK;
}


NTC_StatusTypeDef NTC_ConvertAdcToResistance(NTC_HandleTypeDef *hntc,
                                             uint16_t rawAdcValue,
                                             double *pResistance)
{
    if ((hntc == NULL) || (pResistance == NULL))
    {
        return NTC_ERROR;
    }
    if (hntc->State == NTC_STATE_RESET)
    {
        return NTC_ERROR;
    }

    // Voltage across NTC is (ADC / MaxAdcValue) * SupplyVoltage
    double v_ntc = ( (double)rawAdcValue / (double)hntc->Init.MaxAdcValue )
                  * hntc->Init.SupplyVoltage;

    // 	  Assume a simple voltage divider.

    //	  According to schematic??
    //    Supply --- Rref ---|--- NTC --- GND

    //    The ADC measures V_NTC:
    //    R_NTC = (v_ntc / (V_supply - v_ntc)) * R_ref

    double v_supply_minus_ntc = hntc->Init.SupplyVoltage - v_ntc;
    if (fabsf(v_supply_minus_ntc) < 1e-6f)
    {
        // Avoid division by zero
        return NTC_ERROR;
    }

    double r_ntc = (v_ntc / v_supply_minus_ntc) * hntc->Init.RefResistorValue;

    // Store result
    *pResistance = r_ntc;
    hntc->lastResistance = r_ntc;

    return NTC_OK;
}

NTC_StatusTypeDef NTC_ResistanceToTemperature(NTC_HandleTypeDef *hntc,
                                              double resistance,
                                              double *pTemperature)
{
    if ((hntc == NULL) || (pTemperature == NULL))
    {
        return NTC_ERROR;
    }
    if ((hntc->Init.R25 <= 0.0f) || (hntc->Init.Bvalue <= 0.0f))
    {
        return NTC_ERROR;
    }

    // 2. Apply Beta Equation (from datasheet/AI):
    //    1 / T(K) = 1 / T25 + (1 / B) * ln(R / R25)
    //    T(K) = 1 / [ (1/T25) + (1/B) * ln(R/R25) ]
    //    Then T(°C) = T(K) - 273.15
    // Note: T25 typically 298.15 K for 25°C, R25 is nominal thermistor R at 25°C
    double lnRatio = logf(resistance / hntc->Init.R25);
    double invT = (1.0f / hntc->Init.T25) + (lnRatio / hntc->Init.Bvalue);
    if (fabsf(invT) < 1e-12f)
    {
        // Avoid division by zero
        return NTC_ERROR;
    }

    double tempK = 1.0f / invT;
    double tempC = tempK - 273.15f;

    // Store the result
    *pTemperature = tempC;
    hntc->lastTemperatureC = tempC;

    return NTC_OK;
}

NTC_StatusTypeDef NTC_ReadTemperatureC(NTC_HandleTypeDef *hntc, double *pTemperature)
{
    if ((hntc == NULL) || (pTemperature == NULL))
    {
        return NTC_ERROR;
    }
    if (hntc->State == NTC_STATE_RESET)
    {
        return NTC_ERROR;
    }

    // Read raw ADC
    uint16_t rawVal = 0;
    NTC_StatusTypeDef status = NTC_ReadRawAdc(hntc, &rawVal);
    if (status != NTC_OK)
    {
        return status;
    }

    double r_ntc = 0.0f;
    status = NTC_ConvertAdcToResistance(hntc, rawVal, &r_ntc);
    if (status != NTC_OK)
    {
        return status;
    }

    double tempC = 0.0f;
    status = NTC_ResistanceToTemperature(hntc, r_ntc, &tempC);
    if (status != NTC_OK)
    {
        return status;
    }

    *pTemperature = tempC;

    return NTC_OK;
}




