/*
 * tmp.c
 *
 *  Created on: Nov 22, 2024
 *      Author: John
 */

#include "tmp.h"

TMP_1075_HandleTypeDef h_tmp_1075;

TMP_100_HandleTypeDef h_tmp_100_a;
TMP_100_HandleTypeDef h_tmp_100_b;

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
}

void MX_TMP_100_Init()
{
    h_tmp_100_a.__hi2c = &hi2c2;
    h_tmp_100_a.a0_pin = 1;
    h_tmp_100_a.a1_pin = 1;

    h_tmp_100_a.temp = 0.0;
    h_tmp_100_a.low_limit = 0.0;
    h_tmp_100_a.high_limit = 70.0;

    h_tmp_100_a.conf.os = 0;
    h_tmp_100_a.conf.faults = 0; // 1 fault
    h_tmp_100_a.conf.res = 0b11; // 12 bits
    h_tmp_100_a.conf.sd = 0;

    h_tmp_100_b = h_tmp_100_a;
    h_tmp_100_b.a0_pin = 0;

    if (TMP_100_Init(&h_tmp_100_a) != TMP_100_OK)
    {
        Error_Handler();
    }

    if (TMP_100_Init(&h_tmp_100_b) != TMP_100_OK)
    {
        Error_Handler();
    }
}
