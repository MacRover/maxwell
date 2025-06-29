/*
 * stm32f1xx_at24c04c.c
 *
 *  Created on: June 24, 2024
 *      Author: Ishan
 */

#include "stm32f1xx_at24c04c.h"

AT24C04C_StatusTypeDef AT24C04C_Init(AT24C04C_HandleTypeDef *hat24c04c)
{
    // check at24c04c handle allocation
    if (hat24c04c == NULL)
    {
        return AT24C04C_ERROR;
    }

    if (hat24c04c->State == AT24C04C_STATE_READY)
    {
        // Peripheral is already initialized
        return AT24C04C_ERROR;
    }

    if (hat24c04c->State == AT24C04C_STATE_BUSY)
    {
        return AT24C04C_BUSY;
    }

    if (hat24c04c->State == AT24C04C_STATE_ERROR)
    {
        return AT24C04C_ERROR;
    }

    if (hat24c04c->Init.I2C_HandlerInstance == NULL)
    {
        return AT24C04C_ERROR;
    }

    hat24c04c->State = AT24C04C_STATE_READY;

    // return status
    return AT24C04C_OK;
}

AT24C04C_StatusTypeDef AT24C04C_DeInit(AT24C04C_HandleTypeDef *hat24c04c)
{
    // check at24c04c handle allocation
    if (hat24c04c == NULL)
    {
        return AT24C04C_ERROR;
    }
    // check driver state
    if (hat24c04c->State == AT24C04C_STATE_RESET)
    {
        // Peripheral is not initialized
        return AT24C04C_ERROR;
    }

    if (hat24c04c->State == AT24C04C_STATE_BUSY)
    {
        return AT24C04C_BUSY;
    }

    if (hat24c04c->State == AT24C04C_STATE_ERROR)
    {
        return AT24C04C_ERROR;
    }

    // set driver state
    hat24c04c->State = AT24C04C_STATE_RESET;

    return AT24C04C_OK;
}

AT24C04C_StatusTypeDef AT24C04C_ReadData(AT24C04C_HandleTypeDef *hat24c04c, uint16_t address, uint8_t *buffer, uint16_t len_bytes)
{
    // check at24c04c handle allocation
    if (hat24c04c == NULL)
    {
        return AT24C04C_ERROR;
    }
    // check eepromt state
    if (hat24c04c->State == AT24C04C_STATE_RESET)
    {
        // Peripheral is not initialized
        return AT24C04C_ERROR;
    }

    if (hat24c04c->State == AT24C04C_STATE_BUSY)
    {
        return AT24C04C_BUSY;
    }

    if (hat24c04c->State == AT24C04C_STATE_ERROR)
    {
        return AT24C04C_ERROR;
    }
    // set driver state
    hat24c04c->State = AT24C04C_STATE_BUSY;

    uint16_t I2C_Status = __mem_read(hat24c04c, address, buffer, len_bytes);

    hat24c04c->State = AT24C04C_STATE_READY;

    if (I2C_Status == HAL_I2C_ERROR_TIMEOUT)
    {
        return AT24C04C_TIMEOUT;
    }
    else if (I2C_Status != HAL_I2C_ERROR_NONE)
    {
        return AT24C04C_ERROR;
    }

    return AT24C04C_OK;
}

AT24C04C_StatusTypeDef AT24C04C_WriteData(AT24C04C_HandleTypeDef *hat24c04c, uint16_t address, uint8_t *data, uint16_t len_bytes)
{
    // check at24c04c handle allocation
    if (hat24c04c == NULL)
    {
        return AT24C04C_ERROR;
    }
    // check EEPROM state
    if (hat24c04c->State == AT24C04C_STATE_RESET)
    {
        // Peripheral is not initialized
        return AT24C04C_ERROR;
    }

    if (hat24c04c->State == AT24C04C_STATE_BUSY)
    {
        return AT24C04C_BUSY;
    }

    if (hat24c04c->State == AT24C04C_STATE_ERROR)
    {
        return AT24C04C_ERROR;
    }

    // set driver state
    hat24c04c->State = AT24C04C_STATE_BUSY;

   uint16_t I2C_Status = __mem_write(hat24c04c, address, data, len_bytes);

   hat24c04c->State = AT24C04C_STATE_READY;

    if (I2C_Status == HAL_I2C_ERROR_TIMEOUT)
    {
        return AT24C04C_TIMEOUT;
    }
    else if (I2C_Status != HAL_I2C_ERROR_NONE)
    {
        return AT24C04C_ERROR;
    }

    return AT24C04C_OK;
}

AT24C04C_StatusTypeDef AT24C04C_WritePages(AT24C04C_HandleTypeDef *hat24c04c, uint8_t* pData, uint32_t size, uint8_t eeprom_page_num) {
	uint16_t eeprom_address = eeprom_page_num * hat24c04c->Init.pages;
	uint16_t pageCounter = 0;
	uint16_t address;
	uint16_t currentSize;

    uint8_t bytesPerPage = hat24c04c->Init.page_size - 1;

	uint8_t* data = (uint8_t*) malloc(bytesPerPage);

    uint16_t status;

	do {
		// Setting the EEPROM address based on what page we are on
		address = eeprom_address + pageCounter * hat24c04c->Init.page_size;

		// Checking if the page counter is equivalent to the size passed in, divided by the EEPROM page size
		// This is integer division
		// If we are passed the last whole multiple of EEPROM Page Size, this will evaluate to true
		if (pageCounter == (size / (bytesPerPage))) {
			// Calculate the number of items left to pass in, and set that to the current size

			currentSize = size - (pageCounter * (bytesPerPage));

		} else {
			// If we are not passed the last whole multiple of EEPROM page size, set the data size to EEPROM page size
			currentSize = bytesPerPage;
		}

		// Copy elements from pData to data for use
		memcpy((void *) &data[0], (void *) &pData[pageCounter * (bytesPerPage)], currentSize);


        status = __mem_write(hat24c04c, address, &data[0], currentSize);

        if (status != HAL_I2C_ERROR_NONE)
        {
            break;
        }
		// Incrementing the page counter
		pageCounter++;

	} while (pageCounter <= (size / (bytesPerPage)));
	// Deallocating memory needed
	free((void*) data);

    if (status != HAL_I2C_ERROR_NONE)
    {
        return AT24C04C_ERROR;
    }

    return AT24C04C_OK;
}

AT24C04C_StatusTypeDef AT24C04C_ReadPages(AT24C04C_HandleTypeDef *hat24c04c, uint8_t* pData, uint32_t size, uint8_t eeprom_page_num) {
	uint16_t eeprom_address = eeprom_page_num * hat24c04c->Init.pages;
	uint16_t pageCounter = 0;
	uint16_t address;
	uint16_t currentSize;

    uint8_t bytesPerPage = hat24c04c->Init.page_size - 1;

	uint8_t* data = (uint8_t*) malloc(bytesPerPage);
    uint16_t status;

	do {
		// Setting the EEPROM address based on what page we are on
		address = eeprom_address + pageCounter * hat24c04c->Init.page_size;

		// Checking if the page counter is equivalent to the size passed in, divided by the EEPROM page size
		// This is integer division
		// If we are passed the last whole multiple of EEPROM Page Size, this will evaluate to true
		if (pageCounter == (size / (bytesPerPage))) {
			// Calculate the number of items left to pass in, and set that to the current size
			currentSize = size - (pageCounter * (bytesPerPage));

		} else {
			// If we are not passed the last whole multiple of EEPROM page size, set the data size to EEPROM page size
			currentSize = bytesPerPage;
		}

        status = __mem_read(hat24c04c, address, data, currentSize);

        if (status != HAL_I2C_ERROR_NONE)
        {
            break;
        }

		memcpy((void *) &pData[pageCounter * (bytesPerPage)], (void *) &data[0], currentSize);

		// Incrementing the page counter
		pageCounter++;

	} while (pageCounter <= (size / (bytesPerPage)));

	free((void*) data);

    if (status != HAL_I2C_ERROR_NONE)
    {
        return AT24C04C_ERROR;
    }

    return AT24C04C_OK;
};



uint16_t __mem_read(AT24C04C_HandleTypeDef *hat24c04c, uint16_t address, uint8_t *buffer, uint16_t len_bytes)
{

    uint8_t i2c_address = hat24c04c->Init.device_identifier | (hat24c04c->Init.a2_pin << 3) | (hat24c04c->Init.a1_pin << 2) | (((address & 0x1FF) >> 8) << 1) | 1; // limit eeprom address to 9 bits, get eeprom addr MSB, read command

    while (HAL_I2C_Mem_Read(hat24c04c->Init.I2C_HandlerInstance, (uint16_t)(i2c_address & 0xFE), (uint16_t) (address & 0xFF), I2C_MEMADD_SIZE_8BIT, buffer, len_bytes, 10000) != HAL_OK)
    {

        uint16_t I2C_Error = HAL_I2C_GetError(hat24c04c->Init.I2C_HandlerInstance);

        if (I2C_Error != HAL_I2C_ERROR_AF)
        {
            return I2C_Error;
        }
    }

    return HAL_I2C_ERROR_NONE;

}

uint16_t __mem_write(AT24C04C_HandleTypeDef *hat24c04c, uint16_t address, uint8_t *data, uint16_t len_bytes)
{

    uint8_t i2c_address = hat24c04c->Init.device_identifier | (hat24c04c->Init.a2_pin << 3) | (hat24c04c->Init.a1_pin << 2) | (((address & 0x1FF) >> 8) << 1) | 0; // limit eeprom address to 9 bits, get eeprom addr MSB, write command

    while (HAL_I2C_Mem_Write(hat24c04c->Init.I2C_HandlerInstance, (uint16_t)(i2c_address & 0xFE), (uint16_t) (address & 0xFF), I2C_MEMADD_SIZE_8BIT, data, len_bytes, 10000) != HAL_OK)
    {
        uint16_t I2C_Error = HAL_I2C_GetError(hat24c04c->Init.I2C_HandlerInstance);

        if (I2C_Error != HAL_I2C_ERROR_AF)
        {
            return I2C_Error;
        }
    }

    return HAL_I2C_ERROR_NONE;

}

