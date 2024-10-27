#include "rad_control/rad.hpp"


RAD::RAD(CANraw* can_msg) : l_can_msg(can_msg), l_offset(0.0), l_factor(1.0) { }

RAD::RAD(CANraw* can_msg, uint8_t can_id) : l_can_msg(can_msg), l_offset(0.0), l_factor(1.0)
{
    l_can_msg->address = (uint32_t)can_id;
}

float __buffer_get_float32(uint8_t* buf, uint8_t* ind)
{
#ifndef BIG_ENDIANNESS
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
	return *((float*) &res);
}

double __buffer_get_float64(uint8_t* buf, uint8_t* ind)
{
#ifndef BIG_ENDIANNESS
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
	return *((double*) &res);
}


void __buffer_append_float32(uint8_t* buf, float n, uint8_t* ind)
{
    uint32_t* n_ptr = (uint32_t*) &n;
#ifdef BIG_ENDIANNESS
    *n_ptr = __bswap_32(*n_ptr);
#endif
    buf[(*ind)++] = (*n_ptr & 0xff000000) >> 24;
    buf[(*ind)++] = (*n_ptr & 0x00ff0000) >> 16;
    buf[(*ind)++] = (*n_ptr & 0x0000ff00) >> 8;
    buf[(*ind)++] = (*n_ptr & 0x000000ff);
}

void __buffer_append_float64(uint8_t* buf, double n, uint8_t* ind)
{
    uint64_t* n_ptr = (uint64_t*) &n;
#ifdef BIG_ENDIANNESS
    *n_ptr = __bswap_64(*n_ptr);
#endif
    buf[(*ind)++] = (*n_ptr & 0xff00000000000000) >> 56;
    buf[(*ind)++] = (*n_ptr & 0xff000000000000) >> 48;
    buf[(*ind)++] = (*n_ptr & 0xff0000000000) >> 40;
    buf[(*ind)++] = (*n_ptr & 0xff00000000) >> 32;
    buf[(*ind)++] = (*n_ptr & 0xff000000) >> 24;
    buf[(*ind)++] = (*n_ptr & 0xff0000) >> 16;
    buf[(*ind)++] = (*n_ptr & 0xff00) >> 8;
    buf[(*ind)++] = (*n_ptr & 0xff);
}


uint8_t decode_can_msg(const CANraw* can_msg, RadStatus* status)
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
            status->rad_state = buf[3];
            status->ls_state = !((bool)buf[4]);
            break;
        default:
            return 0;
    }
    return 1;
}

void RAD::set_can_id(uint8_t can_id)
{
    l_can_msg->address = (l_can_msg->address & ~(0xff)) | can_id;
    uint8_t buf[8];
    buf[7] = can_id;
    l_can_msg->address = (l_can_msg->address & 0xff) | ((uint32_t)(CAN_ASSIGN_DEVICE_ID) << 8);
    _update_can_data(buf, 8);
}

void RAD::set_pid_angle_offset(double offset_angle)
{
    this->l_offset = offset_angle;
}

void RAD::set_mul_factor(double factor)
{
    this->l_factor = factor;
}

void RAD::calibrate_zero_pos()
{
    uint8_t buf[1];
    buf[0] = 0;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         (l_can_msg->address & 0xff) | ((uint32_t)(CAN_CALIBRATE_POS) << 8);
    _update_can_data(buf, 1);
}

void RAD::set_target_angle(double angle)
{
    uint8_t ind = 0;
    uint8_t buf[8];
    __buffer_append_float64(buf, l_factor*(angle + l_offset), &ind);
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         (l_can_msg->address & 0xff) | ((uint32_t)(CAN_SET_TARGET_ANGLE) << 8);
    _update_can_data(buf, 8);
}

void RAD::set_stepper_speed(float speed)
{
    uint8_t ind = 0;
    uint8_t buf[4];
    __buffer_append_float32(buf, speed, &ind);
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         (l_can_msg->address & 0xff) | ((uint32_t)(CAN_SET_STEPPER_SPEED) << 8);
    _update_can_data(buf, 4);
}

void RAD::set_p_value(float P)
{
    uint8_t ind = 0;
    uint8_t buf[4];
    __buffer_append_float32(buf, P, &ind);
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         (l_can_msg->address & 0xff) | ((uint32_t)(CAN_SET_P_VALUE) << 8);
    _update_can_data(buf, 4);
}

void RAD::set_i_value(float I)
{
    uint8_t ind = 0;
    uint8_t buf[4];
    __buffer_append_float32(buf, I, &ind);
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         (l_can_msg->address & 0xff) | ((uint32_t)(CAN_SET_I_VALUE) << 8);
    _update_can_data(buf, 4);
}

void RAD::set_d_value(float D)
{
    uint8_t ind = 0;
    uint8_t buf[4];
    __buffer_append_float32(buf, D, &ind);
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         (l_can_msg->address & 0xff) | ((uint32_t)(CAN_SET_D_VALUE) << 8);
    _update_can_data(buf, 4);
}

void RAD::get_target_angle()
{
    uint8_t buf[1];
    buf[0] = 0;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         (l_can_msg->address & 0xff) | ((uint32_t)(CAN_GET_TARGET_ANGLE) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_p_value()
{
    uint8_t buf[1];
    buf[0] = 0;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         (l_can_msg->address & 0xff) | ((uint32_t)(CAN_GET_P_VALUE) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_i_value()
{
    uint8_t buf[1];
    buf[0] = 0;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         (l_can_msg->address & 0xff) | ((uint32_t)(CAN_GET_I_VALUE) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_d_value()
{
    uint8_t buf[1];
    buf[0] = 0;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         (l_can_msg->address & 0xff) | ((uint32_t)(CAN_GET_D_VALUE) << 8);
    _update_can_data(buf, 1);
}


void RAD::_update_can_data(uint8_t* buf, size_t size)
{
    (l_can_msg->data).assign(buf, buf + size);
}