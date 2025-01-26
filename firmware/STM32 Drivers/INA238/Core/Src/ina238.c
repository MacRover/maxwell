


#include "ina238.h"

INA_238_HandleTypeDef ina238_card0_a;
INA_238_HandleTypeDef ina238_card0_b;

INA_238_HandleTypeDef ina238_card2;

void MX_INA_238_Init()
{

    //CARD 0 A
    ina238_card0_a.Init.I2C_HandlerInstance;//
    ina238_card0_a.Init.a0_pin;
    ina238_card0_a.Init.a1_pin;
    ina238_card0_a.Init.device_identifier;
    ina238_card0_a.Init.shunt_resistor;
    ina238_card0_a.Init.max_expected_current;

    ina238_card0_a.ConfigurationRegisters.CONFIG;
    ina238_card0_a.ConfigurationRegisters.ADC_CONFIG;
    ina238_card0_a.ConfigurationRegisters.SHUNT_CAL;
    ina238_card0_a.ConfigurationRegisters.DIAG_ALERT;
    ina238_card0_a.ConfigurationRegisters.SOVL;
    ina238_card0_a.ConfigurationRegisters.SUVL;
    ina238_card0_a.ConfigurationRegisters.BOVL;
    ina238_card0_a.ConfigurationRegisters.BUVL;
    ina238_card0_a.ConfigurationRegisters.PWR_LIMIT;

    //CARD 0 B
    //Only 1 change - address pin for I2C
    ina238_card0_b.Init = ina238_card0_a.Init;
    ina238_card0_b.Init.a1_pin = 1;

    ina238_card0_b.ConfigurationRegisters = ina238_card0_a.ConfigurationRegisters;

    //CARD 2 
    //Change to high power settings

    ina238_card2.Init = ina238_card0_a.Init;
    ina238_card2.Init.shunt_resistor;
    ina238_card2.Init.max_expected_current;

    ina238_card2.ConfigurationRegisters = ina238_card0_a.ConfigurationRegisters;

    ina238_card2.ConfigurationRegisters.SHUNT_CAL;
    ina238_card2.ConfigurationRegisters.SOVL;

    if (INA_238_Init(&ina238_card0_a) != INA_238_OK)
    {
        Error_Handler();
    }

    if (INA_238_Init(&ina238_card0_b) != INA_238_OK)
    {
        Error_Handler();
    }

    if (INA_238_Init(&ina238_card2) != INA_238_OK)
    {
        Error_Handler();
    }


}
