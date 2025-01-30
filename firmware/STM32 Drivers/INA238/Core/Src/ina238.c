#include "ina238.h"
#include "i2c.h"

INA_238_HandleTypeDef ina238_card0_a;
INA_238_HandleTypeDef ina238_card0_b;

INA_238_HandleTypeDef ina238_card2;

void MX_INA_238_Init()
{

    // CARD 0 A
    ina238_card0_a.Init.I2C_HandlerInstance = &hi2c2;
    ina238_card0_a.Init.a0_pin = 0;
    ina238_card0_a.Init.a1_pin = 0;
    ina238_card0_a.Init.device_identifier = 0b10000000;
    ina238_card0_a.Init.shunt_resistor = 1.3E-3;
    ina238_card0_a.Init.max_expected_current = 30;

    ina238_card0_a.ConfigurationRegisters.CONFIG = 0x0010;
    ina238_card0_a.ConfigurationRegisters.ADC_CONFIG = 0xBB6D;
    ina238_card0_a.ConfigurationRegisters.SHUNT_CAL = 0x0F3C;
    ina238_card0_a.ConfigurationRegisters.DIAG_ALERT = 0x0001;
    ina238_card0_a.ConfigurationRegisters.SOVL = 0x79E0;
    ina238_card0_a.ConfigurationRegisters.SUVL = 0x0000;
    ina238_card0_a.ConfigurationRegisters.BOVL = 0x79E0;
    ina238_card0_a.ConfigurationRegisters.BUVL = 0x0000;
    ina238_card0_a.ConfigurationRegisters.PWR_LIMIT = 0xFFFF;

    // CARD 0 B
    // Only 1 change - address pin for I2C
    ina238_card0_b.Init = ina238_card0_a.Init;
    ina238_card0_b.Init.a0_pin = 1;

    ina238_card0_b.ConfigurationRegisters = ina238_card0_a.ConfigurationRegisters;

    // CARD 2
    // Change to high power settings

    ina238_card2.Init = ina238_card0_a.Init;
    ina238_card2.Init.shunt_resistor = 0.7E-3;
    ina238_card2.Init.max_expected_current = 58;

    ina238_card2.ConfigurationRegisters = ina238_card0_a.ConfigurationRegisters;

    ina238_card2.ConfigurationRegisters.SHUNT_CAL = 0x0FDC;
    ina238_card2.ConfigurationRegisters.SOVL = 0x7EE0;

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
