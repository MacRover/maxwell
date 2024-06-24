/*
 * tmc_2590.c
 *
 *  Created on: May 31, 2024
 *      Author: Ethan
 */

#include "tmc_2590.h"

#include "spi.h"
#include "tim.h"

TMC_2590_HandleTypeDef tmc_2590_1;

void MX_TMC_2590_1_Init(void)
{
    tmc_2590_1.Init.SPI_HandlerInstance = &hspi1;
    tmc_2590_1.Init.CS_GPIO_Port = DRIVER_CS_GPIO_Port;
    tmc_2590_1.Init.CS_Pin = DRIVER_CS_Pin;
    tmc_2590_1.Init.ENN_GPIO_Port = DRIVER_ENN_GPIO_Port;
    tmc_2590_1.Init.ENN_Pin = DRIVER_ENN_Pin;
    tmc_2590_1.Init.use_st_alone = 0;
    tmc_2590_1.Init.ST_ALONE_GPIO_Port = DRIVER_ST_ALONE_GPIO_Port;
    tmc_2590_1.Init.ST_ALONE_Pin = DRIVER_ST_ALONE_Pin;
    tmc_2590_1.Init.DIR_GPIO_Port = DRIVER_DIR_GPIO_Port;
    tmc_2590_1.Init.DIR_Pin = DRIVER_DIR_Pin;
    tmc_2590_1.Init.use_pwm = 1;
//    tmc_2590_1.Init.STEP_GPIO_Port = ;
//    tmc_2590_1.Init.STEP_Pin = ;
    tmc_2590_1.Init.STEP_Tim = &htim2;
    tmc_2590_1.Init.STEP_Channel = TIM_CHANNEL_2;
    tmc_2590_1.Init.SG_TST_GPIO_Port = DRIVER_SG_TEST_GPIO_Port;
    tmc_2590_1.Init.SG_TST_Pin = DRIVER_SG_TEST_Pin;

    // this data should stored in EEPROM
    tmc_2590_1.ConfRegisters.CHOPCONF.chm = 0b0;
    tmc_2590_1.ConfRegisters.CHOPCONF.hdec = 0b00;
    tmc_2590_1.ConfRegisters.CHOPCONF.hend = 0b0100;
    tmc_2590_1.ConfRegisters.CHOPCONF.hstrt = 0b110;
    tmc_2590_1.ConfRegisters.CHOPCONF.rndtf = 0b0;
    tmc_2590_1.ConfRegisters.CHOPCONF.tbl = 0b10;
    tmc_2590_1.ConfRegisters.CHOPCONF.toff = 0b100;

    tmc_2590_1.ConfRegisters.DRVCONF.dis_s2g = 0b0;
    tmc_2590_1.ConfRegisters.DRVCONF.en_pfd = 0b1;
    tmc_2590_1.ConfRegisters.DRVCONF.en_s2vs = 0b1;
    tmc_2590_1.ConfRegisters.DRVCONF.otsens = 0b0;
    tmc_2590_1.ConfRegisters.DRVCONF.rdsel = 0b11;
    tmc_2590_1.ConfRegisters.DRVCONF.sdoff = 0b0;
    tmc_2590_1.ConfRegisters.DRVCONF.shrtsens = 0b1;
    tmc_2590_1.ConfRegisters.DRVCONF.slp = 0b11110;
    tmc_2590_1.ConfRegisters.DRVCONF.ts2g = 0b00;
    tmc_2590_1.ConfRegisters.DRVCONF.tst = 0b0;
    tmc_2590_1.ConfRegisters.DRVCONF.vsense = 0b0;

    tmc_2590_1.ConfRegisters.DRVCTRL.dedge = 0b0;
    tmc_2590_1.ConfRegisters.DRVCTRL.intpol = 0b1;
    tmc_2590_1.ConfRegisters.DRVCTRL.mres = 0b1000;

    tmc_2590_1.ConfRegisters.SGCSCONF.cs = 5;
    tmc_2590_1.ConfRegisters.SGCSCONF.sfilt = 0b0;
    tmc_2590_1.ConfRegisters.SGCSCONF.sgt = 0b0000010;

    tmc_2590_1.ConfRegisters.SMARTEN.sedn = 0b00;
    tmc_2590_1.ConfRegisters.SMARTEN.seimin = 0b0;
    tmc_2590_1.ConfRegisters.SMARTEN.semax = 0b0000;
    tmc_2590_1.ConfRegisters.SMARTEN.semin = 0b0000;
    tmc_2590_1.ConfRegisters.SMARTEN.seup = 0b00;

    if (TMC_2590_Init(&tmc_2590_1) != TMC_2590_OK)
    {
        Error_Handler();
    }
}

