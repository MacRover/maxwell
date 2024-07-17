/*
 * stm32f1xx_as5048a.h
 *
 *  Created on: June 15, 2024
 *      Author: Ishan
 */

#ifndef AS5048A_INC_STM32F1XX_AS5048A_H_
#define AS5048A_INC_STM32F1XX_AS5048A_H_

#include "stm32f1xx.h"

/**
 * @brief  AS5048A Status structures definition
 */
typedef enum
{
    AS5048A_OK = 0x00U,
    AS5048A_ERROR = 0x01U,
    AS5048A_BUSY = 0x02U,
    AS5048A_TIMEOUT = 0x03U
} AS5048A_StatusTypeDef;

/**
 * @brief  AS5048A Configuration Structure definition
 */
typedef struct
{
    SPI_HandleTypeDef *SPI_HandlerInstance; // spi instance
    GPIO_TypeDef *CS_GPIO_Port; // cs gpio port
    uint16_t CS_Pin; // cs gpio pin
} AS5048A_InitTypeDef;

/**
 * @brief  AS5048A State structure definition
 */
typedef enum
{
    AS5048A_STATE_RESET = 0x00U, /*!< Peripheral not Initialized                         */
    AS5048A_STATE_READY = 0x01U, /*!< Peripheral Initialized and ready for use           */
    AS5048A_STATE_BUSY = 0x02U, /*!< an internal process is ongoing                     */
    AS5048A_STATE_ERROR = 0x06U, /*!< SPI error state                                    */
} AS5048A_StateTypeDef;

/**
 * @brief  AS5048A handle Structure definition
 */
typedef struct __AS5048A_HandleTypeDef
{
    AS5048A_InitTypeDef Init; /*!< SPI communication parameters*/

    volatile AS5048A_StateTypeDef State; /*!< AS5048A state*/

    volatile uint16_t Angle; /*!< AS5048A Angle*/

    volatile double Angle_double;

    volatile uint16_t MagneticField; /*!< AS5048A Magnetic Field Diagnostic*/

    // todo add error codes
    volatile uint32_t ErrorCode; /*!< AS5048A Error code*/
} AS5048A_HandleTypeDef;

/* Initialization/de-initialization functions  ********************************/
AS5048A_StatusTypeDef AS5048A_Init(AS5048A_HandleTypeDef *has5048a);
AS5048A_StatusTypeDef AS5048A_DeInit(AS5048A_HandleTypeDef *has5048a);

// Read Raw Magnetic Field value
AS5048A_StatusTypeDef AS5048A_ReadAngle(AS5048A_HandleTypeDef *has5048a);

// Read Magnetic Field Diagnostic Value
AS5048A_StatusTypeDef AS5048A_ReadMagneticField(AS5048A_HandleTypeDef *has5048a);

AS5048A_StatusTypeDef __read_angle_command(AS5048A_HandleTypeDef *has5048a);
AS5048A_StatusTypeDef __read_magneticfield_command(
        AS5048A_HandleTypeDef *has5048a);

void __word_to_spi_order_buffer_2bytes(uint16_t word, uint8_t *buff);

uint16_t __spi_order_buffer_to_word_2bytes(uint8_t *buff);

HAL_StatusTypeDef __send_spi_packet_as5048a(AS5048A_HandleTypeDef *has5048a,
        uint16_t SPImsg, uint8_t *SPI_read_bytes);

#endif /* AS5048A_INC_STM32F1XX_AS5048A_H_ */
