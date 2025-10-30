/*
 * tmp_100.c
 *
 *  Created on: Nov 18, 2024
 *      Author: John
 */
#include "tmp_100.h"

/**
 * TMP_100_Init
 *
 * Initialize TMP 1075 handle
 */
TMP_100_StatusTypeDef TMP_100_Init(TMP_100_HandleTypeDef* tmp_100)
{
    if (tmp_100 == NULL)
    {
        return TMP_100_ERROR;
    }
    if (tmp_100->__hi2c == NULL)
    {
        return TMP_100_ERROR;
    }
    return TMP_100_OK;
}

/**
 * TMP_100_DeInit
 *
 * Uninitialize TMP 1075 handle
 */
TMP_100_StatusTypeDef TMP_100_DeInit(TMP_100_HandleTypeDef* tmp_100)
{
    tmp_100->__hi2c = NULL;
    return TMP_100_OK;
}

/**
 * TMP_100_SetLowLimit
 *
 * Sets low temperature limit reading of TMP 1075
 */
TMP_100_StatusTypeDef TMP_100_SetLowLimit(TMP_100_HandleTypeDef* tmp_100)
{
    uint8_t buf[3];
    HAL_StatusTypeDef ret;

    float llt = (tmp_100->low_limit);
    // Bound temperature limit
    if (llt > MAX_TEMP_C)
    {
        llt = MAX_TEMP_C;
    }
    else if (llt < MIN_TEMP_C)
    {
        llt = MIN_TEMP_C;
    }

    // Transmit 12 bits for LLIM
    buf[0] = TMP_100_LLIM;
    buf[1] = (((int16_t)(llt * __scale_factor(tmp_100)) & 0xff0) >> 4);
    buf[2] = (((int16_t)(llt * __scale_factor(tmp_100)) & 0xf) << 4);

    ret = HAL_I2C_Master_Transmit(tmp_100->__hi2c, TMP_100_ADDR | (tmp_100->a1_pin << 3) | (tmp_100->a0_pin << 2), buf, 3, 1000);
    if (ret == HAL_ERROR)
    {
        return TMP_100_ERROR;
    }
    if (ret == HAL_BUSY)
    {
        return TMP_100_BUSY;
    }
    if (ret == HAL_TIMEOUT)
    {
        return TMP_100_TIMEOUT;
    }
    return TMP_100_OK;
}

/**
 * TMP_100_SetHighLimit
 *
 * Sets high temperature limit reading of TMP 1075
 */
TMP_100_StatusTypeDef TMP_100_SetHighLimit(TMP_100_HandleTypeDef* tmp_100)
{
    uint8_t buf[3];
    HAL_StatusTypeDef ret;

    float hlt = (tmp_100->high_limit);
    // Bound temperature limit
    if (hlt > MAX_TEMP_C)
    {
        hlt = MAX_TEMP_C;
    }
    else if (hlt < MIN_TEMP_C)
    {
        hlt = MIN_TEMP_C;
    }

    // Transmit 12 bits for HLIM
    buf[0] = TMP_100_HLIM;
    buf[1] = (((int16_t)(hlt * __scale_factor(tmp_100)) & 0xff0) >> 4);
    buf[2] = (((int16_t)(hlt * __scale_factor(tmp_100)) & 0xf) << 4);

    ret = HAL_I2C_Master_Transmit(tmp_100->__hi2c, TMP_100_ADDR | (tmp_100->a1_pin << 3) | (tmp_100->a0_pin << 2), buf, 3, 1000);
    if (ret == HAL_ERROR)
    {
        return TMP_100_ERROR;
    }
    if (ret == HAL_BUSY)
    {
        return TMP_100_BUSY;
    }
    if (ret == HAL_TIMEOUT)
    {
        return TMP_100_TIMEOUT;
    }
    return TMP_100_OK;
}

/**
 * TMP_100_ReadTemp
 *
 * Reads temperature (in C) of TMP 1075
 */
TMP_100_StatusTypeDef TMP_100_ReadTemp(TMP_100_HandleTypeDef* tmp_100)
{
    uint8_t buf[2];
    HAL_StatusTypeDef ret;

    buf[0] = TMP_100_TEMP;
    ret = HAL_I2C_Master_Transmit(tmp_100->__hi2c, TMP_100_ADDR | (tmp_100->a1_pin << 3) | (tmp_100->a0_pin << 2), buf, 1, 1000);
    if (ret == HAL_ERROR)
    {
        return TMP_100_ERROR;
    }
    if (ret == HAL_BUSY)
    {
        return TMP_100_BUSY;
    }
    if (ret == HAL_TIMEOUT)
    {
        return TMP_100_TIMEOUT;
    }

    ret = HAL_I2C_Master_Receive(tmp_100->__hi2c, TMP_100_ADDR | (tmp_100->a1_pin << 3) | (tmp_100->a0_pin << 2), buf, 2, 1000);
    if (ret == HAL_ERROR)
    {
        return TMP_100_ERROR;
    }
    if (ret == HAL_BUSY)
    {
        return TMP_100_BUSY;
    }
    if (ret == HAL_TIMEOUT)
    {
        return TMP_100_TIMEOUT;
    }

    int16_t raw_temp = (int16_t)( (buf[1] >> 4) | ((uint16_t)buf[0] << 4) );

    // convert to 2's complement for int16
    if (raw_temp & 0x800)
    {
        raw_temp |= (0xf000);
    }
    tmp_100->temp = raw_temp / __scale_factor(tmp_100); // Convert to Celsius

    return TMP_100_OK;
}

/**
 * TMP_100_SetConfRegisters
 *
 * Sets configuration register of TMP 1075
 */
TMP_100_StatusTypeDef TMP_100_SetConfRegisters(TMP_100_HandleTypeDef* tmp_100)
{
    uint8_t buf[2];
    HAL_StatusTypeDef ret;

    uint8_t conf_reg = 0;

    conf_reg |= ((tmp_100->conf.os & 0x01) << 7); // Used only in shutdown mode, returns 0 when read
    conf_reg |= ((tmp_100->conf.res & 0x03) << 5); // Conversation rate set to 250 ms
    conf_reg |= ((tmp_100->conf.faults & 0x03) << 3);
    conf_reg |= ((tmp_100->conf.sd & 0x01) << 0); // If shutdown enabled, set os to 1 to trigger a single conversion

    buf[0] = TMP_100_CFGR;
    buf[1] = conf_reg;

    ret = HAL_I2C_Master_Transmit(tmp_100->__hi2c, TMP_100_ADDR | (tmp_100->a1_pin << 3) | (tmp_100->a0_pin << 2), buf, 1, 1000);
    if (ret == HAL_ERROR)
    {
        return TMP_100_ERROR;
    }
    if (ret == HAL_BUSY)
    {
        return TMP_100_BUSY;
    }
    if (ret == HAL_TIMEOUT)
    {
        return TMP_100_TIMEOUT;
    }
    return TMP_100_OK;
}

double __scale_factor(TMP_100_HandleTypeDef* tmp_100)
{
    switch(tmp_100->conf.res)
    {
        case 0b00:
            return 2.0;
        case 0b01:
            return 4.0;
        case 0b10:
            return 8.0;
        case 0b11:
            return 16.0;
        default:
        	break;
    }

    return 16.0;
}
