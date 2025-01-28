#include "viper_control/viper.hpp"


VIPER::VIPER(CANraw* can_msg) : l_can_msg(can_msg), l_offset(0.0), l_factor(1.0) { }

VIPER::VIPER(CANraw* can_msg, uint8_t can_id) : l_can_msg(can_msg), l_offset(0.0), l_factor(1.0)
{
    l_can_id = (uint32_t)can_id;
}

float __buffer_get_float32(uint8_t* buf, uint8_t* ind)
{
    uint32_t res = __buffer_get_uint32(buf, ind);
	return *((float*) &res);
}

double __buffer_get_float64(uint8_t* buf, uint8_t* ind)
{
    uint64_t res = __buffer_get_uint64(buf, ind);
	return *((double*) &res);
}

uint32_t __buffer_get_uint32(uint8_t* buf, uint8_t* ind)
{
#ifdef BIG_ENDIANNESS
	uint32_t res = ((uint32_t) buf[*ind]) << 24 |
				 ((uint32_t) buf[*ind + 1]) << 16 |
				 ((uint32_t) buf[*ind + 2]) << 8 |
				 ((uint32_t) buf[*ind + 3]);
#else
    uint32_t res = ((uint32_t) buf[*ind]) |
                ((uint32_t) buf[*ind + 1]) << 8 |
                ((uint32_t) buf[*ind + 2]) << 16 |
                ((uint32_t) buf[*ind + 3] << 24);
#endif
	*ind += 4;
    return res;
}

uint64_t __buffer_get_uint64(uint8_t* buf, uint8_t* ind)
{
#ifdef BIG_ENDIANNESS
	uint64_t res = ((uint64_t) buf[*ind]) << 56 |
				 ((uint64_t) buf[*ind + 1]) << 48 |
				 ((uint64_t) buf[*ind + 2]) << 40 |
				 ((uint64_t) buf[*ind + 3]) << 32 |
                 ((uint64_t) buf[*ind + 4]) << 24 |
                 ((uint64_t) buf[*ind + 5]) << 16 |
                 ((uint64_t) buf[*ind + 6]) << 8 |
                 ((uint64_t) buf[*ind + 7]);
#else
    uint64_t res = ((uint64_t) buf[*ind]) |
                ((uint64_t) buf[*ind + 1]) << 8 |
                ((uint64_t) buf[*ind + 2]) << 16 |
                ((uint64_t) buf[*ind + 3]) << 24 | 
                ((uint64_t) buf[*ind + 4]) << 32 | 
                ((uint64_t) buf[*ind + 5]) << 40 | 
                ((uint64_t) buf[*ind + 6]) << 48 | 
                ((uint64_t) buf[*ind + 7]) << 56;
#endif
	*ind += 8;
    return res;
}


void __buffer_append_float32(uint8_t* buf, float n, uint8_t* ind)
{
    uint32_t* n_ptr = (uint32_t*) &n;

    __buffer_append_uint32(buf, *n_ptr, ind);
}

void __buffer_append_float64(uint8_t* buf, double n, uint8_t* ind)
{
    uint64_t* n_ptr = (uint64_t*) &n;

    __buffer_append_uint64(buf, *n_ptr, ind);
}

void __buffer_append_uint32(uint8_t* buf, uint32_t n, uint8_t* ind)
{
#ifndef BIG_ENDIANNESS
    n = __bswap_32(n);
#endif
    buf[(*ind)++] = (n & 0xff000000) >> 24;
    buf[(*ind)++] = (n & 0x00ff0000) >> 16;
    buf[(*ind)++] = (n & 0x0000ff00) >> 8;
    buf[(*ind)++] = (n & 0x000000ff);
}

void __buffer_append_uint64(uint8_t* buf, uint64_t n, uint8_t* ind)
{
#ifndef BIG_ENDIANNESS
    n = __bswap_64(n);
#endif
    buf[(*ind)++] = (n & 0xff00000000000000) >> 56;
    buf[(*ind)++] = (n & 0xff000000000000) >> 48;
    buf[(*ind)++] = (n & 0xff0000000000) >> 40;
    buf[(*ind)++] = (n & 0xff00000000) >> 32;
    buf[(*ind)++] = (n & 0xff000000) >> 24;
    buf[(*ind)++] = (n & 0xff0000) >> 16;
    buf[(*ind)++] = (n & 0xff00) >> 8;
    buf[(*ind)++] = (n & 0xff);
}

uint8_t decode_can_msg(const CANraw* can_msg, VIPERStatus* status)
{
    uint8_t* buf = (uint8_t*) &(can_msg->data[0]);
    uint8_t i = 0;

    switch(((can_msg->address) >> 8) & 0xff)
    {
        case CAN_SEND_ODOM_ANGLE:
            status->angle = __buffer_get_float64(buf, &i);
            break;
        case CAN_SEND_HEALTH_STATUS:
            status->eeprom_status = buf[0];
            status->tmc_status = buf[1];
            status->encoder_status = buf[2];
            status->VIPER_state = buf[3];
            status->ls_state = (bool)buf[4];
            break;
        default:
            return 0;
    }
    return 1;
}

void VIPER::set_can_id(uint8_t can_id)
{
    uint8_t buf[1];
    buf[0] = can_id;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                        ((uint32_t)l_can_id) | ((uint32_t)(CAN_ASSIGN_DEVICE_ID) << 8);
    _update_can_data(buf, 1);
    // Set VIPER ID after sending command
    this->l_can_id = can_id;
}

void VIPER::save_to_eeprom()
{
    uint8_t buf[1];
    buf[0] = 0;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SAVE_TO_EEPROM) << 8);
    _update_can_data(buf, 1);
}


void VIPER::set_health_interval(uint16_t health_interval)
{
    uint8_t ind = 0;
    uint8_t buf[2];
    __buffer_append_uint32(buf, period, &ind);
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_HEALTH_INTERVAL) << 8);
    _update_can_data(buf, 2);
}

void VIPER::set_card_interval(uint16_t card_interval)
{
    uint8_t ind = 0;
    uint8_t buf[2];
    __buffer_append_uint32(buf, period, &ind);
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_HEALTH_INTERVAL) << 8);
    _update_can_data(buf, 2);
}






