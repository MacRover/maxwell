/*
 * as5048a.c
 *
 *  Created on: June 15, 2024
 *      Author: Ishan
 */

#include "as5048a.h"

#include "spi.h"


AS5048A_HandleTypeDef as5048a_2;

void MX_AS5048A_2_Init(void)
{
    as5048a_2.Init.SPI_HandlerInstance = &hspi2;
    as5048a_2.Init.CS_GPIO_Port = ENCODER_CS_GPIO_Port;
    as5048a_2.Init.CS_Pin = ENCODER_CS_Pin;


    if (AS5048A_Init(&as5048a_2) != AS5048A_OK)
    {
        Error_Handler();
    }
}

