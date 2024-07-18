/*
 * as5048a.c
 *
 *  Created on: Jun 23, 2024
 *      Author: Ethan
 */

#include "as5048a.h"

#include "spi.h"

AS5048A_HandleTypeDef as5048a_1;

void MX_AS5048A_1_Init(void)
{
    as5048a_1.Init.CS_GPIO_Port = ENCODER_CS_GPIO_Port;
    as5048a_1.Init.CS_Pin = ENCODER_CS_Pin;
    as5048a_1.Init.SPI_HandlerInstance = &hspi2;

    if (AS5048A_Init(&as5048a_1) != AS5048A_OK)
    {
        Error_Handler();
    }
    // flush out undefined angle value
    AS5048A_ReadAngle(&as5048a_1);
    AS5048A_ReadAngle(&as5048a_1);
    AS5048A_ReadAngle(&as5048a_1);
}
