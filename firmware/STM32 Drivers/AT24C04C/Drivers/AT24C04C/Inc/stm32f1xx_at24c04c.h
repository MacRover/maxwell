/*
 * stm32f1xx_at24c04c.h
 *
 *  Created on: June 24, 2024
 *      Author: Ishan
 */

#ifndef AT24C04C_INC_STM32F1XX_AT24C04C_H_
#define AT24C04C_INC_STM32F1XX_AT24C04C_H_

#include "stm32f1xx.h"

#include <stdlib.h>
#include <string.h>

/**
 * @brief  AT24C04C Status structures definition
 */
typedef enum
{
    AT24C04C_OK = 0x00U,
    AT24C04C_ERROR = 0x01U,
    AT24C04C_BUSY = 0x02U,
    AT24C04C_TIMEOUT = 0x03U
} AT24C04C_StatusTypeDef;


/**
 * @brief  AT24C04C Configuration Structure definition
 */
typedef struct
{
    I2C_HandleTypeDef *I2C_HandlerInstance; // i2c instance

    uint8_t device_identifier;

    uint8_t a2_pin;

    uint8_t a1_pin;

    uint8_t page_size;

    uint8_t pages;
} AT24C04C_InitTypeDef;

/**
 * @brief  AT24C04C State structure definition
 */
typedef enum
{
    AT24C04C_STATE_RESET = 0x00U, /*!< Peripheral not Initialized                         */
    AT24C04C_STATE_READY = 0x01U, /*!< Peripheral Initialized and ready for use           */
    AT24C04C_STATE_BUSY = 0x02U, /*!< an internal process is ongoing                     */
    AT24C04C_STATE_ERROR = 0x06U, /*!< SPI error state                                    */
} AT24C04C_StateTypeDef;

/**
 * @brief  AT24C04C handle Structure definition
 */
typedef struct __AT24C04C_HandleTypeDef
{
    AT24C04C_InitTypeDef Init; /*!< SPI communication parameters*/

    volatile AT24C04C_StateTypeDef State; /*!< AT24C04C state*/

    // todo add error codes
    volatile uint32_t ErrorCode; /*!< AT24C04C Error code*/
} AT24C04C_HandleTypeDef;

/* Initialization/de-initialization functions  ********************************/
AT24C04C_StatusTypeDef AT24C04C_Init(AT24C04C_HandleTypeDef *hat24c04c);
AT24C04C_StatusTypeDef AT24C04C_DeInit(AT24C04C_HandleTypeDef *hat24c04c);

// Write Data to EEPROM
AT24C04C_StatusTypeDef AT24C04C_WriteData(AT24C04C_HandleTypeDef *hat24c04c, uint16_t address, uint8_t *data, uint16_t len_bytes);

// Read Data from EEPROM
AT24C04C_StatusTypeDef AT24C04C_ReadData(AT24C04C_HandleTypeDef *hat24c04c, uint16_t address, uint8_t *buffer, uint16_t len_bytes);

//Write to multiple pages of the EEPROM
AT24C04C_StatusTypeDef AT24C04C_WritePages(AT24C04C_HandleTypeDef *hat24c04c, uint8_t* pData, uint32_t size, uint8_t eeprom_page_num);

//Read From multiple pages of the EEPROM
AT24C04C_StatusTypeDef AT24C04C_ReadPages(AT24C04C_HandleTypeDef *hat24c04c, uint8_t* pData, uint32_t size, uint8_t eeprom_page_num);


uint16_t __mem_read(AT24C04C_HandleTypeDef *hat24c04c, uint16_t address, uint8_t *buffer, uint16_t len_bytes);

uint16_t __mem_write(AT24C04C_HandleTypeDef *hat24c04c, uint16_t address, uint8_t *data, uint16_t len_bytes);

#endif /* AT24C04C_INC_STM32F1XX_AT24C04C_H_ */
