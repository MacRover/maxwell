/*
 * stm32f1xx_as5048a.c
 *
 *  Created on: June 15, 2024
 *      Author: Ishan
 */

#include "stm32f1xx_as5048a.h"

AS5048A_StatusTypeDef AS5048A_Init(AS5048A_HandleTypeDef *has5048a)
{
    // check as5048a handle allocation
    if (has5048a == NULL)
    {
        return AS5048A_ERROR;
    }

    if (has5048a->State == AS5048A_STATE_READY)
    {
        // Peripheral is already initialized
        return AS5048A_ERROR;
    }

    if (has5048a->State == AS5048A_STATE_BUSY)
    {
        return AS5048A_BUSY;
    }

    if (has5048a->State == AS5048A_STATE_ERROR)
    {
        return AS5048A_ERROR;
    }

    if (has5048a->Init.SPI_HandlerInstance == NULL)
    {
        return AS5048A_ERROR;
    }

    if (!has5048a->Init.CS_GPIO_Port)
    {
        return AS5048A_ERROR;
    }

    has5048a->State = AS5048A_STATE_READY;

    // return status
    return AS5048A_OK;
}

AS5048A_StatusTypeDef AS5048A_DeInit(AS5048A_HandleTypeDef *has5048a)
{
    // check as5048a handle allocation
    if (has5048a == NULL)
    {
        return AS5048A_ERROR;
    }
    // check driver state
    if (has5048a->State == AS5048A_STATE_RESET)
    {
        // Peripheral is not initialized
        return AS5048A_ERROR;
    }

    if (has5048a->State == AS5048A_STATE_BUSY)
    {
        return AS5048A_BUSY;
    }

    if (has5048a->State == AS5048A_STATE_ERROR)
    {
        return AS5048A_ERROR;
    }

    // set driver state
    has5048a->State = AS5048A_STATE_RESET;

    return AS5048A_OK;
}

AS5048A_StatusTypeDef AS5048A_ReadAngle(AS5048A_HandleTypeDef *has5048a)
{
    // check as5048a handle allocation
    if (has5048a == NULL)
    {
        return AS5048A_ERROR;
    }
    // check driver state
    if (has5048a->State == AS5048A_STATE_RESET)
    {
        // Peripheral is not initialized
        return AS5048A_ERROR;
    }

    if (has5048a->State == AS5048A_STATE_BUSY)
    {
        return AS5048A_BUSY;
    }

    if (has5048a->State == AS5048A_STATE_ERROR)
    {
        return AS5048A_ERROR;
    }
    // set driver state
    has5048a->State = AS5048A_STATE_BUSY;

    if (__read_angle_command(has5048a) != AS5048A_OK)
    {
        //todo remove this it's a hack
        has5048a->State = AS5048A_STATE_READY;
        return AS5048A_ERROR;
    }

    has5048a->State = AS5048A_STATE_READY;

    return AS5048A_OK;
}

AS5048A_StatusTypeDef AS5048A_ReadMagneticField(AS5048A_HandleTypeDef *has5048a)
{
    // check as5048a handle allocation
    if (has5048a == NULL)
    {
        return AS5048A_ERROR;
    }
    // check driver state
    if (has5048a->State == AS5048A_STATE_RESET)
    {
        // Peripheral is not initialized
        return AS5048A_ERROR;
    }

    if (has5048a->State == AS5048A_STATE_BUSY)
    {
        return AS5048A_BUSY;
    }

    if (has5048a->State == AS5048A_STATE_ERROR)
    {
        return AS5048A_ERROR;
    }

    // set driver state
    has5048a->State = AS5048A_STATE_BUSY;

    if (__read_magneticfield_command(has5048a) != AS5048A_OK)
    {
        //todo remove this it's a hack
        has5048a->State = AS5048A_STATE_READY;
        return AS5048A_ERROR;
    }

    has5048a->State = AS5048A_STATE_READY;

    return AS5048A_OK;
}

AS5048A_StatusTypeDef __read_angle_command(AS5048A_HandleTypeDef *has5048a)
{
    uint16_t SPImsg = 0xFFFF;
    uint16_t SPIread;
    uint8_t SPI_read_bytes[2];

    if (__send_spi_packet_as5048a(has5048a, SPImsg, SPI_read_bytes) != HAL_OK)
    {
        // todo handle error
        return AS5048A_ERROR;
    }

    SPIread = __spi_order_buffer_to_word_2bytes(SPI_read_bytes);

    uint16_t angle_data = SPIread & 0x3fff;
    uint8_t error = (SPIread & 0x4000) >> 14;

    if (error)
    {
        __send_spi_packet_as5048a(has5048a, 0x4001, SPI_read_bytes);
        return AS5048A_ERROR;
    }

    // fast parity check
    uint16_t parity_check = SPIread;
    parity_check ^= parity_check >> 8;
    parity_check ^= parity_check >> 4;
    parity_check ^= parity_check >> 2;
    parity_check ^= parity_check >> 1;
    uint8_t parity = (~parity_check) & 1;
    if (!parity)
    {
        return AS5048A_ERROR;
    }

    has5048a->Angle = angle_data;
    has5048a->Angle_double = ((double) angle_data) * (360.0 / 16383.0);

    return AS5048A_OK;
}

AS5048A_StatusTypeDef __read_magneticfield_command(
        AS5048A_HandleTypeDef *has5048a)
{
    uint16_t SPImsg = 0x7FFD;
    uint16_t SPIread;
    uint8_t SPI_read_bytes[2];

    if (__send_spi_packet_as5048a(has5048a, SPImsg, SPI_read_bytes) != HAL_OK)
    {
        // todo handle error
        return AS5048A_ERROR;
    }

    SPIread = __spi_order_buffer_to_word_2bytes(SPI_read_bytes);

    has5048a->MagneticField = SPIread;

    return AS5048A_OK;
}

void __word_to_spi_order_buffer_2bytes(uint16_t word, uint8_t *buff)
{
    buff[0] = (uint8_t) ((word & 0xFF00) >> 8);
    buff[1] = (uint8_t) (word & 0xFF);
}

uint16_t __spi_order_buffer_to_word_2bytes(uint8_t *buff)
{
    return ((uint16_t) buff[0] << 8) | (uint16_t) buff[1];
}

HAL_StatusTypeDef __send_spi_packet_as5048a(AS5048A_HandleTypeDef *has5048a,
        uint16_t SPImsg, uint8_t *SPI_read_bytes)
{
    uint8_t SPImsg_bytes[2];
    __word_to_spi_order_buffer_2bytes(SPImsg, SPImsg_bytes);
    // write new registers
    HAL_GPIO_WritePin(has5048a->Init.CS_GPIO_Port, has5048a->Init.CS_Pin,
            GPIO_PIN_RESET);
    HAL_StatusTypeDef spi_status = HAL_SPI_TransmitReceive(
            has5048a->Init.SPI_HandlerInstance, SPImsg_bytes, SPI_read_bytes, 2,
            1000);
    HAL_GPIO_WritePin(has5048a->Init.CS_GPIO_Port, has5048a->Init.CS_Pin,
            GPIO_PIN_SET);
    return spi_status;
}
