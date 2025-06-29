/*
 * stm32f1xx_tmc_2590.c
 *
 *  Created on: May 31, 2024
 *      Author: Ethan
 */

#include "stm32f1xx_tmc_2590.h"

#include <stdlib.h>

TMC_2590_StatusTypeDef TMC_2590_Init(TMC_2590_HandleTypeDef *htmc2590)
{
    // check tmc2590 handle allocation
    if (htmc2590 == NULL)
    {
        return TMC_2590_ERROR;
    }

    if (htmc2590->State == TMC_2590_STATE_READY)
    {
        // Peripheral is already initialized
        return TMC_2590_ERROR;
    }

    if (htmc2590->State == TMC_2590_STATE_BUSY)
    {
        return TMC_2590_BUSY;
    }

    if (htmc2590->State == TMC_2590_STATE_ERROR)
    {
        return TMC_2590_ERROR;
    }

    if (htmc2590->Init.SPI_HandlerInstance == NULL)
    {
        return TMC_2590_ERROR;
    }

    if (!htmc2590->Init.use_pwm && htmc2590->Init.STEP_Tim == NULL)
    {
        return TMC_2590_ERROR;
    }
    // set driver state
    htmc2590->State = TMC_2590_STATE_BUSY;

    // todo throw errors if pins are undefined

    // set st_alone
    HAL_GPIO_WritePin(htmc2590->Init.ST_ALONE_GPIO_Port,
            htmc2590->Init.ST_ALONE_Pin,
            htmc2590->Init.use_st_alone ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // set default pin state. this should actually already be configured by the gpio library
    if (!htmc2590->Init.use_st_alone)
    {
        // todo check conf registers
        HAL_GPIO_WritePin(htmc2590->Init.CS_GPIO_Port, htmc2590->Init.CS_Pin,
                GPIO_PIN_SET);
        // write config registers
        __send_conf_registers(htmc2590);
    }
    if (!htmc2590->Init.use_pwm)
    {
        // set default step
        HAL_GPIO_WritePin(htmc2590->Init.STEP_GPIO_Port,
                htmc2590->Init.STEP_Pin, GPIO_PIN_RESET);
    }
    // set default dir
    HAL_GPIO_WritePin(htmc2590->Init.DIR_GPIO_Port, htmc2590->Init.DIR_Pin,
            GPIO_PIN_RESET);

    // set enn
    HAL_GPIO_WritePin(htmc2590->Init.ENN_GPIO_Port, htmc2590->Init.ENN_Pin,
            GPIO_PIN_RESET);

    if (htmc2590->Init.use_pwm)
    {
        // pre-allocate memory for pwm dma
        htmc2590->__pwm_dma_ptr = (uint16_t*) calloc(htmc2590->Init.max_steps,
                sizeof(uint16_t));
        // check calloc was successful
        if (htmc2590->__pwm_dma_ptr == NULL)
        {
            htmc2590->State = TMC_2590_STATE_ERROR;
            return TMC_2590_ERROR;
        }
        // set all values to 50% duty cycle
        for (uint16_t i = 0; i < htmc2590->Init.max_steps; i++)
        {
            htmc2590->__pwm_dma_ptr[i] = 50;
        }
    }
    // set driver state
    htmc2590->State = TMC_2590_STATE_READY;

    // return status
    return TMC_2590_OK;
}

TMC_2590_StatusTypeDef TMC_2590_DeInit(TMC_2590_HandleTypeDef *htmc2590)
{
    // check tmc2590 handle allocation
    if (htmc2590 == NULL)
    {
        return TMC_2590_ERROR;
    }
    // check driver state
    if (htmc2590->State == TMC_2590_STATE_RESET)
    {
        // Peripheral is not initialized
        return TMC_2590_ERROR;
    }

    if (htmc2590->State == TMC_2590_STATE_BUSY)
    {
        return TMC_2590_BUSY;
    }

    if (htmc2590->State == TMC_2590_STATE_ERROR)
    {
        return TMC_2590_ERROR;
    }

    // unset enn
    HAL_GPIO_WritePin(htmc2590->Init.ENN_GPIO_Port, htmc2590->Init.ENN_Pin,
            GPIO_PIN_SET);

    free(htmc2590->__pwm_dma_ptr);

    // set driver state
    htmc2590->State = TMC_2590_STATE_RESET;

    return TMC_2590_OK;
}

//Used to indicate if the driver is free or not
TMC_2590_StatusTypeDef TMC_2590_CheckState(TMC_2590_HandleTypeDef *htmc2590)
{
    if (htmc2590 == NULL)
    {
        return TMC_2590_ERROR;
    }
    // check driver state
    if (htmc2590->State == TMC_2590_STATE_RESET)
    {
        // Peripheral is not initialized
        return TMC_2590_ERROR;
    }

    if (htmc2590->State == TMC_2590_STATE_BUSY)
    {
        return TMC_2590_BUSY;
    }

    if (htmc2590->State == TMC_2590_STATE_ERROR)
    {
        return TMC_2590_ERROR;
    }

    return TMC_2590_OK;
}

TMC_2590_StatusTypeDef TMC_2590_WriteConfRegisters(
        TMC_2590_HandleTypeDef *htmc2590)
{
    // check tmc2590 handle allocation
    if (htmc2590 == NULL)
    {
        return TMC_2590_ERROR;
    }
    // check driver state
    if (htmc2590->State == TMC_2590_STATE_RESET)
    {
        // Peripheral is not initialized
        return TMC_2590_ERROR;
    }

    if (htmc2590->State == TMC_2590_STATE_BUSY)
    {
        return TMC_2590_BUSY;
    }

    if (htmc2590->State == TMC_2590_STATE_ERROR)
    {
        return TMC_2590_ERROR;
    }

    // TODO check conf registers
    
    TMC_2590_StateTypeDef set_register_error;

    // set driver state
    htmc2590->State = TMC_2590_STATE_BUSY;
    set_register_error = __send_conf_registers(htmc2590);

    htmc2590->State = TMC_2590_STATE_READY;

    return set_register_error;
}

TMC_2590_StatusTypeDef TMC_2590_MoveSteps(TMC_2590_HandleTypeDef *htmc2590, int16_t steps)
{
    // check tmc2590 handle allocation
    if (htmc2590 == NULL)
    {
        return TMC_2590_ERROR;
    }
    // check driver state
    if (htmc2590->State == TMC_2590_STATE_RESET)
    {
        // Peripheral is not initialized
        return TMC_2590_ERROR;
    }

    if (htmc2590->State == TMC_2590_STATE_BUSY)
    {
        return TMC_2590_BUSY;
    }

    if (htmc2590->State == TMC_2590_STATE_ERROR)
    {
        return TMC_2590_ERROR;
    }

    // short-circuit if steps to move is 0, prevents callback to reset state from never firing
    if (steps == 0)
    {
        return TMC_2590_OK;
    }

    // set driver state
    htmc2590->State = TMC_2590_STATE_BUSY;

    // set dir
    if (((steps < 0) && (!htmc2590->Init.inverted)) || ((steps > 0) && (htmc2590->Init.inverted)))
    {
        HAL_GPIO_WritePin(htmc2590->Init.DIR_GPIO_Port, htmc2590->Init.DIR_Pin,
                GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(htmc2590->Init.DIR_GPIO_Port, htmc2590->Init.DIR_Pin,
                GPIO_PIN_SET);
    }

    if (!htmc2590->Init.use_pwm)
    {
        // pulse step pin with some delay
        for (uint32_t i = 0; i < steps; i++)
        {
            // TODO set a constant for the delay
            HAL_Delay(1);
            HAL_GPIO_WritePin(htmc2590->Init.STEP_GPIO_Port,
                    htmc2590->Init.STEP_Pin, GPIO_PIN_SET);
            // TODO set a constant for the delay
            HAL_Delay(1);
            HAL_GPIO_WritePin(htmc2590->Init.STEP_GPIO_Port,
                    htmc2590->Init.STEP_Pin, GPIO_PIN_RESET);
        }
        //  set driver state
        htmc2590->State = TMC_2590_STATE_READY;
        return TMC_2590_OK;
    }


    // callback moves TMC2590 to READY state
    // config timer settings to pulse
    uint16_t pwm_pulses = abs(steps);
    if (pwm_pulses > htmc2590->Init.max_steps)
    {
        // throw error
        htmc2590->State = TMC_2590_STATE_ERROR;
        return TMC_2590_ERROR;
    }

    HAL_TIM_PWM_Start_DMA(htmc2590->Init.STEP_Tim, htmc2590->Init.STEP_Channel,
            (uint32_t*) htmc2590->__pwm_dma_ptr, pwm_pulses);
    return TMC_2590_OK;
}

void TMC_2590_Stop(TMC_2590_HandleTypeDef *htmc2590)
{
    if (htmc2590->State == TMC_2590_STATE_BUSY)
    {
        HAL_TIM_PWM_Stop_DMA(htmc2590->Init.STEP_Tim, htmc2590->Init.STEP_Channel);
        htmc2590->State = TMC_2590_STATE_READY;
    }
}

TMC_2590_StatusTypeDef TMC_2590_SetTimAutoReload(TMC_2590_HandleTypeDef *htmc2590, uint32_t autoreload)
{
    __HAL_TIM_SET_AUTORELOAD(htmc2590->Init.STEP_Tim, autoreload);
    return TMC_2590_OK;
}

TMC_2590_StatusTypeDef TMC_2590_SG_Read(TMC_2590_HandleTypeDef *htmc2590)
{
    // check tmc2590 handle allocation
    if (htmc2590 == NULL)
    {
        return TMC_2590_ERROR;
    }
    // todo check driver status

    // read sg test pin
    htmc2590->sg_test_state = HAL_GPIO_ReadPin(htmc2590->Init.SG_TST_GPIO_Port,
            htmc2590->Init.SG_TST_Pin);

    return TMC_2590_OK;
}

void TMC_2590_TIM_PWM_PulseFinishedCallback(TMC_2590_HandleTypeDef *htmc2590,
        TIM_HandleTypeDef *htim)
{
    HAL_TIM_PWM_Stop_DMA(htim, htmc2590->Init.STEP_Channel);
    htmc2590->State = TMC_2590_STATE_READY;
}

TMC_2590_StatusTypeDef __send_conf_registers(TMC_2590_HandleTypeDef *htmc2590)
{
    uint32_t SPImsg = 0;
    uint8_t SPI_read_bytes[3];

    SPImsg = __TMC_2590_ConfRegister_Header_DRVCONF; // DRVCONF
    SPImsg |= (htmc2590->ConfRegisters.DRVCONF.tst << 16);
    SPImsg |= (htmc2590->ConfRegisters.DRVCONF.slp << 11);
    SPImsg |= (htmc2590->ConfRegisters.DRVCONF.dis_s2g << 10);
    SPImsg |= (htmc2590->ConfRegisters.DRVCONF.ts2g << 8);
    SPImsg |= (htmc2590->ConfRegisters.DRVCONF.sdoff << 7);
    SPImsg |= (htmc2590->ConfRegisters.DRVCONF.vsense << 6);
    SPImsg |= (htmc2590->ConfRegisters.DRVCONF.rdsel << 4);
    SPImsg |= (htmc2590->ConfRegisters.DRVCONF.otsens << 3);
    SPImsg |= (htmc2590->ConfRegisters.DRVCONF.shrtsens << 2);
    SPImsg |= (htmc2590->ConfRegisters.DRVCONF.en_pfd << 1);
    SPImsg |= (htmc2590->ConfRegisters.DRVCONF.en_s2vs << 0);
    // write new registers
    if (__send_spi_packet(htmc2590, SPImsg, SPI_read_bytes) != HAL_OK)
    {
        // todo handle error
        return TMC_2590_ERROR;
    }
    // SPI_read_bytes to DRVSTATUS
    __set_drvstatus_struct(htmc2590, SPI_read_bytes);

    // set rest of the registers
    SPImsg = __TMC_2590_ConfRegister_Header_SGCSCONF;
    SPImsg |= (htmc2590->ConfRegisters.SGCSCONF.sfilt << 16);
    SPImsg |= (htmc2590->ConfRegisters.SGCSCONF.sgt << 8);
    SPImsg |= (htmc2590->ConfRegisters.SGCSCONF.cs << 0);
    if (__send_spi_packet(htmc2590, SPImsg, SPI_read_bytes) != HAL_OK)
    {
        // todo handle error
        return TMC_2590_ERROR;
    }
    __set_drvstatus_struct(htmc2590, SPI_read_bytes);

    SPImsg = __TMC_2590_ConfRegister_Header_SMARTEN;
    SPImsg |= (htmc2590->ConfRegisters.SMARTEN.seimin << 15);
    SPImsg |= (htmc2590->ConfRegisters.SMARTEN.sedn << 13);
    SPImsg |= (htmc2590->ConfRegisters.SMARTEN.semax << 8);
    SPImsg |= (htmc2590->ConfRegisters.SMARTEN.seup << 5);
    SPImsg |= (htmc2590->ConfRegisters.SMARTEN.semin << 0);
    if (__send_spi_packet(htmc2590, SPImsg, SPI_read_bytes) != HAL_OK)
    {
        // todo handle error
        return TMC_2590_ERROR;
    }
    __set_drvstatus_struct(htmc2590, SPI_read_bytes);

    SPImsg = __TMC_2590_ConfRegister_Header_CHOPCONF;
    SPImsg |= (htmc2590->ConfRegisters.CHOPCONF.tbl << 15);
    SPImsg |= (htmc2590->ConfRegisters.CHOPCONF.chm << 14);
    SPImsg |= (htmc2590->ConfRegisters.CHOPCONF.rndtf << 13);
    SPImsg |= (htmc2590->ConfRegisters.CHOPCONF.hdec << 11);
    SPImsg |= (htmc2590->ConfRegisters.CHOPCONF.hend << 7);
    SPImsg |= (htmc2590->ConfRegisters.CHOPCONF.hstrt << 4);
    SPImsg |= (htmc2590->ConfRegisters.CHOPCONF.toff << 0);
    if (__send_spi_packet(htmc2590, SPImsg, SPI_read_bytes) != HAL_OK)
    {
        // todo handle error
        return TMC_2590_ERROR;
    }
    __set_drvstatus_struct(htmc2590, SPI_read_bytes);

    SPImsg = __TMC_2590_ConfRegister_Header_DRVCTRL;
    SPImsg |= (htmc2590->ConfRegisters.DRVCTRL.intpol << 9);
    SPImsg |= (htmc2590->ConfRegisters.DRVCTRL.dedge << 8);
    SPImsg |= (htmc2590->ConfRegisters.DRVCTRL.mres << 0);
    if (__send_spi_packet(htmc2590, SPImsg, SPI_read_bytes) != HAL_OK)
    {
        // todo handle error
        return TMC_2590_ERROR;
    }
    __set_drvstatus_struct(htmc2590, SPI_read_bytes);
    return TMC_2590_OK;
}

void __word_to_spi_order_buffer(uint32_t word, uint8_t *buff)
{
    // convert to big endian
    buff[2] = (uint8_t) (word & 0xFF);
    buff[1] = (uint8_t) ((word & 0xFF00) >> 8);
    buff[0] = (uint8_t) ((word & 0xFF0000) >> 16);
}

uint32_t __spi_order_buffer_to_word(uint8_t *buff)
{
    return ((uint32_t) buff[0] << 16) | ((uint32_t) buff[1] << 8)
            | (uint32_t) buff[2];
}

HAL_StatusTypeDef __send_spi_packet(TMC_2590_HandleTypeDef *htmc2590,
        uint32_t SPImsg, uint8_t *SPI_read_bytes)
{
    uint8_t SPImsg_bytes[3];
    __word_to_spi_order_buffer(SPImsg, SPImsg_bytes);
    // write new registers
    HAL_GPIO_WritePin(htmc2590->Init.CS_GPIO_Port, htmc2590->Init.CS_Pin,
            GPIO_PIN_RESET);
    HAL_StatusTypeDef spi_status = HAL_SPI_TransmitReceive(
            htmc2590->Init.SPI_HandlerInstance, SPImsg_bytes, SPI_read_bytes, 3,
            1000);
    HAL_GPIO_WritePin(htmc2590->Init.CS_GPIO_Port, htmc2590->Init.CS_Pin,
            GPIO_PIN_SET);
    return spi_status;
}

void __set_drvstatus_struct(TMC_2590_HandleTypeDef *htmc2590, uint8_t *status)
{
    uint32_t status_word = __spi_order_buffer_to_word(status);

    htmc2590->DrvStatus.sg = status_word & 0x0001;
    htmc2590->DrvStatus.ot = (status_word >> 1) & 0x0001;
    htmc2590->DrvStatus.otpw = (status_word >> 2) & 0x0001;
    htmc2590->DrvStatus.shorta = (status_word >> 3) & 0x0001;
    htmc2590->DrvStatus.shortb = (status_word >> 4) & 0x0001;
    htmc2590->DrvStatus.ola = (status_word >> 5) & 0x0001;
    htmc2590->DrvStatus.olb = (status_word >> 6) & 0x0001;
    htmc2590->DrvStatus.stst = (status_word >> 7) & 0x0001;
    htmc2590->DrvStatus.unused_bits = (status_word >> 8) & 0x0003;
    htmc2590->DrvStatus.mstep_SGCS_status_diagnostic = (status_word >> 10)
            & 0x03FF;
}
