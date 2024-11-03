/*
 * tmc_2590.c
 *
 *  Created on: May 31, 2024
 *      Author: Ethan
 */

#include "tmc_2590.h"

#include "spi.h"
#include "tim.h"

#include "main.h"

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
    tmc_2590_1.Init.max_steps = 1000;
    tmc_2590_1.Init.SG_TST_GPIO_Port = DRIVER_SG_TEST_GPIO_Port;
    tmc_2590_1.Init.SG_TST_Pin = DRIVER_SG_TEST_Pin;

    // this data should stored in EEPROM
    //These are default params if EEPROM cannot be read
    tmc_2590_1.ConfRegisters.CHOPCONF.chm = rad_params.CHOPCONF_CHM;
    tmc_2590_1.ConfRegisters.CHOPCONF.hdec = rad_params.CHOPCONF_HDEC;
    tmc_2590_1.ConfRegisters.CHOPCONF.hend = rad_params.CHOPCONF_HEND;
    tmc_2590_1.ConfRegisters.CHOPCONF.hstrt = rad_params.CHOPCONF_HSTRT;
    tmc_2590_1.ConfRegisters.CHOPCONF.rndtf = rad_params.CHOPCONF_RNDTF;
    tmc_2590_1.ConfRegisters.CHOPCONF.tbl = rad_params.CHOPCONF_TBL;
    tmc_2590_1.ConfRegisters.CHOPCONF.toff = rad_params.CHOPCONF_TOFF;

    tmc_2590_1.ConfRegisters.DRVCONF.dis_s2g = rad_params.DRVCONF_DIS_S2G;
    tmc_2590_1.ConfRegisters.DRVCONF.en_pfd = rad_params.DRVCONF_EN_PFD;
    tmc_2590_1.ConfRegisters.DRVCONF.en_s2vs = rad_params.DRVCONF_EN_S2VS;
    tmc_2590_1.ConfRegisters.DRVCONF.otsens = rad_params.DRVCONF_OTSENS;
    tmc_2590_1.ConfRegisters.DRVCONF.rdsel = rad_params.DRVCONF_RDSEL;
    tmc_2590_1.ConfRegisters.DRVCONF.sdoff = rad_params.DRVCONF_SDOFF;
    tmc_2590_1.ConfRegisters.DRVCONF.shrtsens = rad_params.DRVCONF_SHRTSENS;
    tmc_2590_1.ConfRegisters.DRVCONF.slp = rad_params.DRVCONF_SLP;
    tmc_2590_1.ConfRegisters.DRVCONF.ts2g = rad_params.DRVCONF_TS2G;
    tmc_2590_1.ConfRegisters.DRVCONF.tst = rad_params.DRVCONF_TST;
    tmc_2590_1.ConfRegisters.DRVCONF.vsense = rad_params.DRVCONF_VSENSE;

    tmc_2590_1.ConfRegisters.DRVCTRL.dedge = rad_params.DRVCTRL_DEDGE;
    tmc_2590_1.ConfRegisters.DRVCTRL.intpol = rad_params.DRVCTRL_INTPOL;
    tmc_2590_1.ConfRegisters.DRVCTRL.mres = rad_params.DRVCTRL_MRES;

    tmc_2590_1.ConfRegisters.SGCSCONF.cs = rad_params.SGCSCONF_CS;
    tmc_2590_1.ConfRegisters.SGCSCONF.sfilt = rad_params.SGCSCONF_SFILT;
    tmc_2590_1.ConfRegisters.SGCSCONF.sgt = rad_params.SGCSCONF_SGT;
    
    tmc_2590_1.ConfRegisters.SMARTEN.sedn = rad_params.SMARTEN_SEDN;
    tmc_2590_1.ConfRegisters.SMARTEN.seimin = rad_params.SMARTEN_SEIMIN;
    tmc_2590_1.ConfRegisters.SMARTEN.semax = rad_params.SMARTEN_SEMAX;
    tmc_2590_1.ConfRegisters.SMARTEN.semin = rad_params.SMARTEN_SEMIN;
    tmc_2590_1.ConfRegisters.SMARTEN.seup = rad_params.SMARTEN_SEUP;

    if (TMC_2590_Init(&tmc_2590_1) != TMC_2590_OK)
    {
        Error_Handler();
    }
}

