/*
 * at24c04c.c
 *
 *  Created on: June 24, 2024
 *      Author: Ishan
 */

#include "at24c04c.h"

#include "i2c.h"


AT24C04C_HandleTypeDef at24c04c_1;

void MX_AT24C04C_1_Init(void)
{
    at24c04c_1.Init.I2C_HandlerInstance = &hi2c1;
    at24c04c_1.Init.device_identifier = 0b10100000;
    at24c04c_1.Init.a2_pin = 0;
    at24c04c_1.Init.a1_pin = 0;


    if (AT24C04C_Init(&at24c04c_1) != AT24C04C_OK)
    {
        Error_Handler();
    }
}

