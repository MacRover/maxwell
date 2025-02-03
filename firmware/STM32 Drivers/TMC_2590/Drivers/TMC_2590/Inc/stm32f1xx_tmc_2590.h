/*
 * stm32f1xx_tmc_2590.h
 *
 *  Created on: May 31, 2024
 *      Author: Ethan
 */

#ifndef TMC_2590_INC_STM32F1XX_TMC_2590_H_
#define TMC_2590_INC_STM32F1XX_TMC_2590_H_

#include "stm32f1xx.h"

/**
 * @brief  TMC 2590 Status structures definition
 */
typedef enum
{
    TMC_2590_OK = 0x00U,
    TMC_2590_ERROR = 0x01U,
    TMC_2590_BUSY = 0x02U,
    TMC_2590_TIMEOUT = 0x03U
} TMC_2590_StatusTypeDef;

typedef struct
{
    uint8_t tst; // Test mode
    uint8_t slp; // Slope control
    uint8_t dis_s2g; // Short to ground protection
    uint8_t ts2g; // Short detection delay
    uint8_t sdoff; // Step/dir interface
    uint8_t vsense; // Full-scale sense resistor voltage setting
    uint8_t rdsel; // Read out select
    uint8_t otsens; // Overtemp shutdown setting
    uint8_t shrtsens; // Short to ground sensitivity
    uint8_t en_pfd; // Passive fast delay setting
    uint8_t en_s2vs; // Short to VS protection
} TMC_2590_DRVCONF_TypeDef;

typedef struct
{
    uint8_t sfilt; // Stall guard filter
    uint8_t sgt; // Stall guard threshold
    uint8_t cs; // Current scale
} TMC_2590_SGCSCONF_TypeDef;

typedef struct
{
    uint8_t seimin; // Min cool step current
    uint8_t sedn; // Current dec. speed
    uint8_t semax; // Upper cool step threshold offset
    uint8_t seup; // Current increment size
    uint8_t semin; // Cool step lower threshold
} TMC_2590_SMARTEN_TypeDef;

typedef struct
{
    uint8_t tbl; // Blanking time
    uint8_t chm; // Chopper mode
    uint8_t rndtf; // Random TOFF time
    uint8_t hdec; // Hysteresis decay or fast decay mode
    uint8_t hend; // Hysteresis end value
    uint8_t hstrt; // Hysteresis start value
    uint8_t toff; // MOSFET off time
} TMC_2590_CHOPCONF_TypeDef;

typedef struct
{
    uint8_t intpol; // Step interpolation
    uint8_t dedge; // Double edge step pulses
    uint8_t mres; // Microsteps per fullstep
} TMC_2590_DRVCTRL_TypeDef;

typedef struct
{
    TMC_2590_DRVCONF_TypeDef DRVCONF; // Driver Configuration Register
    TMC_2590_SGCSCONF_TypeDef SGCSCONF; // StallGuard2 Configuration Register
    TMC_2590_SMARTEN_TypeDef SMARTEN; // CoolStep Configuration Register
    TMC_2590_CHOPCONF_TypeDef CHOPCONF; // Chopper Configuration Register
    TMC_2590_DRVCTRL_TypeDef DRVCTRL; // Driver Control Register
} TMC_2590_ConfRegisters_TypeDef;

typedef enum
{
    __TMC_2590_ConfRegister_Header_DRVCONF = (uint32_t) (0b111 << 17),
    __TMC_2590_ConfRegister_Header_SGCSCONF = (uint32_t) (0b110 << 17),
    __TMC_2590_ConfRegister_Header_SMARTEN = (uint32_t) (0b101 << 17),
    __TMC_2590_ConfRegister_Header_CHOPCONF = (uint32_t) (0b100 << 17),
    __TMC_2590_ConfRegister_Header_DRVCTRL = (uint32_t) (0b00 << 18)
} __TMC_2590_ConfRegister_Header_TypeDef;

typedef struct
{
    uint8_t sg; // StallGuard2 status
    uint8_t ot; // Overtemp shutdown
    uint8_t otpw; // Overtemp warning
    uint8_t shorta; // Short detection status A
    uint8_t shortb; // Short detection status B
    uint8_t ola; // Open load indicator A
    uint8_t olb; // Open load indicator B
    uint8_t stst; // Standstill indicator
    uint8_t unused_bits; // Unused bits, helpful for determining mstep_SGCS_status_diagnostic

    uint16_t mstep_SGCS_status_diagnostic; // Microstep counter/StallGuard2 SG9:0/StallGuard2 SG9:5 and CoolStep SE4:0/Diagnostic status
} TMC_2590_DRVSTATUS_TypeDef;

/**
 * @brief  TMC 2590 Configuration Structure definition
 */
typedef struct
{
    SPI_HandleTypeDef *SPI_HandlerInstance; // spi instance
    GPIO_TypeDef *CS_GPIO_Port; // cs gpio port
    uint16_t CS_Pin; // cs gpio pin

    GPIO_TypeDef *ENN_GPIO_Port; // ENN gpio port
    uint16_t ENN_Pin; // ENN gpio pin

    uint8_t use_st_alone; // boolean determines st_alone mode
    GPIO_TypeDef *ST_ALONE_GPIO_Port; // ST_ALONE gpio port
    uint16_t ST_ALONE_Pin; // ST_ALONE gpio pin

    GPIO_TypeDef *DIR_GPIO_Port; // DIR gpio port
    uint16_t DIR_Pin; // DIR gpio pin

    uint8_t use_pwm; // boolean determines whether driver is operating with GPIO or PWM step control
    GPIO_TypeDef *STEP_GPIO_Port; // step gpio port
    uint16_t STEP_Pin; // step gpio pin
    TIM_HandleTypeDef *STEP_Tim; // step pin timer
    uint32_t STEP_Channel; // step pin channel
    uint16_t max_steps; // maximum number of steps per command in pwm mode. Determines how much memory to allocate.

    GPIO_TypeDef *SG_TST_GPIO_Port; // SG_TST gpio port
    uint16_t SG_TST_Pin; // SG_TST gpio pin

    uint8_t inverted; //Inverts direction
} TMC_2590_InitTypeDef;

/**
 * @brief  TMC 2590 State structure definition
 */
typedef enum
{
    TMC_2590_STATE_RESET = 0x00U, /*!< Peripheral not Initialized                         */
    TMC_2590_STATE_READY = 0x01U, /*!< Peripheral Initialized and ready for use           */
    TMC_2590_STATE_BUSY = 0x02U, /*!< an internal process is ongoing                     */
    TMC_2590_STATE_ERROR = 0x06U, /*!< SPI error state                                    */
} TMC_2590_StateTypeDef;

/**
 * @brief  TMC 2590 handle Structure definition
 */
typedef struct __TMC_2590_HandleTypeDef
{
    TMC_2590_InitTypeDef Init; /*!< SPI communication parameters*/

    TMC_2590_ConfRegisters_TypeDef ConfRegisters;

    // not sure if this actually needs to be volatile
    volatile GPIO_PinState sg_test_state; // SG TEST Pin state

    volatile TMC_2590_DRVSTATUS_TypeDef DrvStatus;

    volatile TMC_2590_StateTypeDef State; /*!< TMC 2590 state*/

    // todo add error codes
    volatile uint32_t ErrorCode; /*!< TMC 2590 Error code*/

    uint16_t *__pwm_dma_ptr; // memory space for PWM DMA
} TMC_2590_HandleTypeDef;

/* Initialization/de-initialization functions  ********************************/
TMC_2590_StatusTypeDef TMC_2590_Init(TMC_2590_HandleTypeDef *htmc2590);
TMC_2590_StatusTypeDef TMC_2590_DeInit(TMC_2590_HandleTypeDef *htmc2590);

TMC_2590_StatusTypeDef TMC_2590_CheckState(TMC_2590_HandleTypeDef *htmc2590);

// write registers
TMC_2590_StatusTypeDef TMC_2590_WriteConfRegisters(
        TMC_2590_HandleTypeDef *htmc2590);

// move X steps
TMC_2590_StatusTypeDef TMC_2590_MoveSteps(TMC_2590_HandleTypeDef *htmc2590,
        int16_t steps);

void TMC_2590_Stop(TMC_2590_HandleTypeDef *htmc2590);

TMC_2590_StatusTypeDef TMC_2590_SetTimAutoReload(TMC_2590_HandleTypeDef *htmc2590, 
		uint32_t autoreload);


// read sg_test?
TMC_2590_StatusTypeDef TMC_2590_SG_Read(TMC_2590_HandleTypeDef *htmc2590);

// TIM callback
void TMC_2590_TIM_PWM_PulseFinishedCallback(TMC_2590_HandleTypeDef *htmc2590,
        TIM_HandleTypeDef *htim);

// read registers? prob don't need this, conf write should handle this

TMC_2590_StatusTypeDef __send_conf_registers(TMC_2590_HandleTypeDef *htmc2590);

void __word_to_spi_order_buffer(uint32_t word, uint8_t *buff);

uint32_t __spi_order_buffer_to_word(uint8_t *buff);

void __set_drvstatus_struct(TMC_2590_HandleTypeDef *htmc2590, uint8_t *status);

HAL_StatusTypeDef __send_spi_packet(TMC_2590_HandleTypeDef *htmc2590,
        uint32_t SPImsg, uint8_t *SPI_read_bytes);

#endif /* TMC_2590_INC_STM32F1XX_TMC_2590_H_ */
