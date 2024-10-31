/*
 * enc_dec_utils.h
 *
 *  Created on: Jul 29, 2024
 *      Author: Ethan
 */

#ifndef INC_ENC_DEC_UTILS_H_
#define INC_ENC_DEC_UTILS_H_

#include <stdint.h>
#include <string.h>


void encode_float_big_endian(float value, uint8_t *data);
void encode_double_big_endian(double value, uint8_t *data);
void encode_uint32_big_endian(uint32_t value, uint8_t *data);
void encode_uint16_big_endian(uint16_t value, uint8_t *data);


float decode_float_big_endian(uint8_t *data);
uint32_t decode_uint32_big_endian(uint8_t *data);
uint16_t decode_uint16_big_endian(uint8_t *data);
double decode_double_big_endian(uint8_t *data);

#endif /* INC_ENC_DEC_UTILS_H_ */
