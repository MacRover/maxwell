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

//    TMC_2590_ConfRegisters_TypeDef tmc_2590_confRegisters;
//    // this data should stored in EEPROM
//    tmc_2590_1.ConfRegisters = tmc_2590_confRegisters;

    if (TMC_2590_Init(&tmc_2590_1) != TMC_2590_OK)
    {
        Error_Handler();
    }
}

