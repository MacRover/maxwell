/*
 * enc_dec_utils.c
 *
 *  Created on: Jul 29, 2024
 *      Author: Ethan
 */

#include "enc_dec_utils.h"

void encode_float_big_endian(float value, uint8_t *data)
{
    uint32_t *fpt_bin_ptr = (uint32_t*) &value;

    data[0] = ((*fpt_bin_ptr) & 0xff000000) >> 24;
    data[1] = ((*fpt_bin_ptr) & 0x00ff0000) >> 16;
    data[2] = ((*fpt_bin_ptr) & 0x0000ff00) >> 8;
    data[3] = (*fpt_bin_ptr) & 0x000000ff;
}

float decode_float_big_endian(uint8_t *data)
{
    uint32_t fpt_bin = (data[0] << 24) | (data[1] << 16) | (data[2] << 8)
            | data[3];
    return *((float*) &fpt_bin);
}

double decode_double_big_endian(uint8_t *data)
{
    uint64_t fpt_bin = (data[0] << 56) | (data[1] << 48) | (data[2] << 40)
            | (data[3] << 32) | (data[4] << 24) | (data[5] << 16)
            | (data[6] << 8) | data[7];
    return *((double*) &fpt_bin);
}
