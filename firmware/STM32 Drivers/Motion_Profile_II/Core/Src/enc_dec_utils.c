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

void encode_double_big_endian(double value, uint8_t *data)
{
    uint64_t *fpt_bin_ptr = (uint64_t*) &value;

    data[0] = ((*fpt_bin_ptr) & 0xff00000000000000) >> 56;
    data[1] = ((*fpt_bin_ptr) & 0x00ff000000000000) >> 48;
    data[2] = ((*fpt_bin_ptr) & 0x0000ff0000000000) >> 40;
    data[3] = ((*fpt_bin_ptr) & 0x000000ff00000000) >> 32;
    data[4] = ((*fpt_bin_ptr) & 0x00000000ff000000) >> 24;
    data[5] = ((*fpt_bin_ptr) & 0x0000000000ff0000) >> 16;
    data[6] = ((*fpt_bin_ptr) & 0x000000000000ff00) >> 8;
    data[7] = (*fpt_bin_ptr) & 0x00000000000000ff;
}

void encode_uint32_big_endian(uint32_t value, uint8_t *data)
{
    uint32_t *fpt_bin_ptr = (uint32_t*) &value;

    data[0] = ((*fpt_bin_ptr) & 0xff000000) >> 24;
    data[1] = ((*fpt_bin_ptr) & 0x00ff0000) >> 16;
    data[2] = ((*fpt_bin_ptr) & 0x0000ff00) >> 8;
    data[3] = (*fpt_bin_ptr) & 0x000000ff;
}

void encode_uint16_big_endian(uint16_t value, uint8_t *data)
{
    uint16_t *fpt_bin_ptr = (uint16_t*) &value;

    data[0] = ((*fpt_bin_ptr) & 0xff00) >> 8;
    data[1] = (*fpt_bin_ptr) & 0x00ff;
}

float decode_float_big_endian(uint8_t *data)
{
    uint32_t fpt_bin = (data[0] << 24) | (data[1] << 16) | (data[2] << 8)
            | data[3];
    return *((float*) &fpt_bin);
}

uint32_t decode_uint32_big_endian(uint8_t *data)
{
    uint32_t fpt_bin = (data[0] << 24) | (data[1] << 16) | (data[2] << 8)
            | data[3];
    return fpt_bin;
}

uint16_t decode_uint16_big_endian(uint8_t *data)
{
    uint32_t fpt_bin = (data[0] << 8) | (data[1]);
    return fpt_bin;
}

double decode_double_big_endian(uint8_t *data)
{
	uint64_t fpt_bin = 0; //floating point bin
    uint32_t bin1 = 0;
    uint32_t bin2 = 0;

    bin1 = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    bin2 = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];

    fpt_bin = (((uint64_t) bin1 & 0xFFFFFFFF) << 32) | bin2;

    return *((double*) &fpt_bin);
}
