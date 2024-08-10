/*
 * enc_dec_utils.h
 *
 *  Created on: Jul 29, 2024
 *      Author: Ethan
 */

#ifndef INC_ENC_DEC_UTILS_H_
#define INC_ENC_DEC_UTILS_H_

#include <stdint.h>

void encode_float_big_endian(float value, uint8_t *data);
float decode_float_big_endian(uint8_t *data);
double decode_double_big_endian(uint8_t *data);

#endif /* INC_ENC_DEC_UTILS_H_ */
