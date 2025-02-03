/*
 * tmp.c
 *
 *  Created on: Nov 22, 2024
 *      Author: John
 */

#include "tmp.h"

TMP_1075_HandleTypeDef h_tmp_1075;

void MX_TMP_1075_Init()
{
    h_tmp_1075.__hi2c = &hi2c2;
    h_tmp_1075.a0_pin = 0;

    h_tmp_1075.temp = 0.0;
    h_tmp_1075.low_limit = 0.0;
    h_tmp_1075.high_limit = 70.0;

    h_tmp_1075.conf.os = 0;
    h_tmp_1075.conf.faults = 0; // 1 fault
    h_tmp_1075.conf.polarity = 0;
    h_tmp_1075.conf.tm = 0;
    h_tmp_1075.conf.sd = 0;

    if (TMP_1075_Init(&h_tmp_1075) != TMP_1075_OK)
    {
        Error_Handler();
    }

    TMP_1075_SetHighLimit(&h_tmp_1075);
    TMP_1075_SetLowLimit(&h_tmp_1075);
    TMP_1075_SetConfRegisters(&h_tmp_1075);
}
