/*
 * as5048a.c
 *
 *  Created on: June 15, 2024
 *      Author: Ishan
 */

#include "as5048a.h"

#include "spi.h"


AS5048A_HandleTypeDef as5048a_1;

void MX_AS5048A_1_Init(void)
{
    as5048a_1.Init.SPI_HandlerInstance = &hspi2;
    as5048a_1.Init.CS_GPIO_Port = ENCODER_CS_GPIO_Port;
    as5048a_1.Init.CS_Pin = ENCODER_CS_Pin;


    if (AS5048A_Init(&as5048a_1) != AS5048A_OK)
    {
        Error_Handler();
    }
}

