/*
 * tmp_1075.c
 *
 *  Created on: Nov 18, 2024
 *      Author: John
 */
#include "tmp_1075.h"

/**
 * TMP_1075_Init
 *
 * Initialize TMP 1075 handle
 */
TMP_1075_StatusTypeDef TMP_1075_Init(TMP_1075_HandleTypeDef* tmp_1075)
{
    if (tmp_1075 == NULL)
    {
        return TMP_1075_ERROR;
    }
    if (tmp_1075->__hi2c == NULL)
    {
        return TMP_1075_ERROR;
    }
    return TMP_1075_OK;
}

/**
 * TMP_1075_DeInit
 *
 * Uninitialize TMP 1075 handle
 */
TMP_1075_StatusTypeDef TMP_1075_DeInit(TMP_1075_HandleTypeDef* tmp_1075)
{
    tmp_1075->__hi2c = NULL;
    return TMP_1075_OK;
}

/**
 * TMP_1075_SetLowLimit
 *
 * Sets low temperature limit reading of TMP 1075
 */
TMP_1075_StatusTypeDef TMP_1075_SetLowLimit(TMP_1075_HandleTypeDef* tmp_1075)
{
    uint8_t buf[3];
    HAL_StatusTypeDef ret;

    float llt = (tmp_1075->low_limit);
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
    buf[0] = TMP_1075_LLIM;
    buf[1] = (((int16_t)(llt * 16.0f) & 0xff0) >> 4);
    buf[2] = (((int16_t)(llt * 16.0f) & 0xf) << 4);

    ret = HAL_I2C_Master_Transmit(tmp_1075->__hi2c, TMP_1075_ADDR | (tmp_1075->a0_pin << 1), buf, 3, 1000);
    if (ret == HAL_ERROR)
    {
        return TMP_1075_ERROR;
    }
    if (ret == HAL_BUSY)
    {
        return TMP_1075_BUSY;
    }
    if (ret == HAL_TIMEOUT)
    {
        return TMP_1075_TIMEOUT;
    }
    return TMP_1075_OK;
}

/**
 * TMP_1075_SetHighLimit
 *
 * Sets high temperature limit reading of TMP 1075
 */
TMP_1075_StatusTypeDef TMP_1075_SetHighLimit(TMP_1075_HandleTypeDef* tmp_1075)
{
    uint8_t buf[3];
    HAL_StatusTypeDef ret;

    float hlt = (tmp_1075->high_limit);
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
    buf[0] = TMP_1075_HLIM;
    buf[1] = (((int16_t)(hlt * 16.0f) & 0xff0) >> 4);
    buf[2] = (((int16_t)(hlt * 16.0f) & 0xf) << 4);

    ret = HAL_I2C_Master_Transmit(tmp_1075->__hi2c, TMP_1075_ADDR | (tmp_1075->a0_pin << 1), buf, 3, 1000);
    if (ret == HAL_ERROR)
    {
        return TMP_1075_ERROR;
    }
    if (ret == HAL_BUSY)
    {
        return TMP_1075_BUSY;
    }
    if (ret == HAL_TIMEOUT)
    {
        return TMP_1075_TIMEOUT;
    }
    return TMP_1075_OK;
}

/**
 * TMP_1075_ReadTemp
 *
 * Reads temperature (in C) of TMP 1075
 */
TMP_1075_StatusTypeDef TMP_1075_ReadTemp(TMP_1075_HandleTypeDef* tmp_1075)
{
    uint8_t buf[2];
    HAL_StatusTypeDef ret;

    buf[0] = TMP_1075_TEMP;
    ret = HAL_I2C_Master_Transmit(tmp_1075->__hi2c, TMP_1075_ADDR | (tmp_1075->a0_pin << 1), buf, 1, 1000);
    if (ret == HAL_ERROR)
    {
        return TMP_1075_ERROR;
    }
    if (ret == HAL_BUSY)
    {
        return TMP_1075_BUSY;
    }
    if (ret == HAL_TIMEOUT)
    {
        return TMP_1075_TIMEOUT;
    }

    ret = HAL_I2C_Master_Receive(tmp_1075->__hi2c, TMP_1075_ADDR | (tmp_1075->a0_pin << 1), buf, 2, 1000);
    if (ret == HAL_ERROR)
    {
        return TMP_1075_ERROR;
    }
    if (ret == HAL_BUSY)
    {
        return TMP_1075_BUSY;
    }
    if (ret == HAL_TIMEOUT)
    {
        return TMP_1075_TIMEOUT;
    }

    int16_t raw_temp = (int16_t)( (buf[1] >> 4) | ((uint16_t)buf[0] << 4) );

    // convert to 2's complement for int16
    if (raw_temp & 0x800)
    {
        raw_temp |= (0xf000);
    }
    tmp_1075->temp = raw_temp / 16.0f; // Convert to Celsius

    return TMP_1075_OK;
}

/**
 * TMP_1075_SetConfRegisters
 *
 * Sets configuration register of TMP 1075
 */
TMP_1075_StatusTypeDef TMP_1075_SetConfRegisters(TMP_1075_HandleTypeDef* tmp_1075)
{
    uint8_t buf[3];
    HAL_StatusTypeDef ret;

    uint8_t conf_reg_high = 0;
    uint8_t conf_reg_low  = 0xa0; // Low byte reserved

    conf_reg_high |= ((tmp_1075->conf.os & 0x01) << 7); // Used only in shutdown mode, returns 0 when read
    conf_reg_high |= (0x03 << 5); // Conversation rate set to 250 ms
    conf_reg_high |= ((tmp_1075->conf.faults & 0x03) << 3);
    conf_reg_high |= ((tmp_1075->conf.polarity & 0x01) << 2);
    conf_reg_high |= ((tmp_1075->conf.tm & 0x01) << 1);
    conf_reg_high |= ((tmp_1075->conf.sd & 0x01) << 0); // If shutdown enabled, set os to 1 to trigger a single conversion

    buf[0] = TMP_1075_CFGR;
    buf[1] = conf_reg_high;
    buf[2] = conf_reg_low;

    ret = HAL_I2C_Master_Transmit(tmp_1075->__hi2c, TMP_1075_ADDR | (tmp_1075->a0_pin << 1), buf, 3, 1000);
    if (ret == HAL_ERROR)
    {
        return TMP_1075_ERROR;
    }
    if (ret == HAL_BUSY)
    {
        return TMP_1075_BUSY;
    }
    if (ret == HAL_TIMEOUT)
    {
        return TMP_1075_TIMEOUT;
    }
    return TMP_1075_OK;
}

