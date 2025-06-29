/*
 * ntcb572.h
 *
 *  Created on: Mar 18, 2025
 *      Author: Ali
 */

#ifndef NTCB572_INC_NTCB572_H_
#define NTCB572_INC_NTCB572_H_


#include "stm32f1xx.h"

// 1. STATUS AND STATE ENUMS

typedef enum
{
    NTC_OK       = 0x00U,
    NTC_ERROR    = 0x01U,
    NTC_BUSY     = 0x02U,
    NTC_TIMEOUT  = 0x03U
} NTC_StatusTypeDef;


typedef enum
{
    NTC_STATE_RESET = 0x00U,
    NTC_STATE_READY = 0x01U,
    NTC_STATE_BUSY  = 0x02U,
    NTC_STATE_ERROR = 0x03U
} NTC_StateTypeDef;


// 2. INITIALIZATION STRUCT

typedef struct
{
    ADC_HandleTypeDef *hadc;      // Pointer to the ADC handle used for readings
    uint32_t           Channel;   // ADC channel for the NTC voltage divider

    double              SupplyVoltage;      // 3.3 V
    double              RefResistorValue;   // 10 kΩ from schematic
    double              Bvalue;             // B-constant from datasheet: 3455 K
    double              R25;                // 10 kΩ
    double              T25;                // 25 °C in Kelvin
    uint16_t           MaxAdcValue;        // 4095 for a 12-bit ADC

} NTC_InitTypeDef;


// 3. HANDLE


typedef struct __NTC_HandleTypeDef
{
    NTC_InitTypeDef    Init;
    volatile NTC_StateTypeDef State;
    volatile uint32_t  ErrorCode;

    double lastResistance;            // Last computed resistance
    double lastTemperatureC;          // Last computed temperature (°C)

} NTC_HandleTypeDef;


// 4. DRIVER FUNCTIONS


NTC_StatusTypeDef NTC_Init(NTC_HandleTypeDef *hntc);

NTC_StatusTypeDef NTC_DeInit(NTC_HandleTypeDef *hntc);

NTC_StatusTypeDef NTC_ReadRawAdc(NTC_HandleTypeDef *hntc, uint16_t *pRawAdcValue);

NTC_StatusTypeDef NTC_ConvertAdcToResistance(NTC_HandleTypeDef *hntc,
                                             uint16_t rawAdcValue,
                                             double *pResistance);

NTC_StatusTypeDef NTC_ResistanceToTemperature(NTC_HandleTypeDef *hntc,
                                              double resistance,
                                              double *pTemperature);

NTC_StatusTypeDef NTC_ReadTemperatureC(NTC_HandleTypeDef *hntc, double *pTemperature);


#endif



