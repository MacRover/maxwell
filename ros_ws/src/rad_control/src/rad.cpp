#include "rad_control/rad.hpp"


RAD::RAD(CANraw* can_msg) : l_can_msg(can_msg) { }

RAD::RAD(CANraw* can_msg, uint8_t can_id) : l_can_msg(can_msg)
{
    l_can_msg->address = (uint32_t)can_id;
}

float __buffer_get_float32(uint8_t* buf, uint8_t* ind)
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
	return *((float*) &res);
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

uint8_t decode_can_msg(CANraw* can_msg, RadStatus* status)
{
    uint8_t* buf = (uint8_t*) &(can_msg->data[0]);
    uint8_t i = 0;

    switch((can_msg->address) >> 8)
    {
        case CAN_STATUS_1:
            status->ls_state = (bool)(buf[0] & 2);
            status->upper_bound_state = (bool)(buf[0] & 1);
            i = i + 1;
            status->angle = __buffer_get_float32(buf, &i);
            break;
        case CAN_STATUS_2:
            status->p = __buffer_get_float32(buf, &i);
            status->i = __buffer_get_float32(buf, &i);
            break;
        case CAN_STATUS_3:
            status->d = __buffer_get_float32(buf, &i);
            status->speed = __buffer_get_float32(buf, &i);
            break;
        default:
            return 0;
    }
    return 1;
}

void RAD::set_target_angle(float angle)
{
    uint8_t ind = 0;
    uint8_t buf[4];
    __buffer_append_float32(buf, angle, &ind);
    l_can_msg->address = (l_can_msg->address & 0xff) | ((uint32_t)(CAN_SET_TARGET_ANGLE) << 8);
    _update_can_data(buf, 4);
}

void RAD::set_stepper_speed(float speed)
{
    uint8_t ind = 0;
    uint8_t buf[4];
    __buffer_append_float32(buf, speed, &ind);
    l_can_msg->address = (l_can_msg->address & 0xff) | ((uint32_t)(CAN_SET_STEPPER_SPEED) << 8);
    _update_can_data(buf, 4);
}

void RAD::set_p_value(float P)
{
    uint8_t ind = 0;
    uint8_t buf[4];
    __buffer_append_float32(buf, P, &ind);
    l_can_msg->address = (l_can_msg->address & 0xff) | ((uint32_t)(CAN_SET_P_VALUE) << 8);
    _update_can_data(buf, 4);
}

void RAD::set_i_value(float I)
{
    uint8_t ind = 0;
    uint8_t buf[4];
    __buffer_append_float32(buf, I, &ind);
    l_can_msg->address = (l_can_msg->address & 0xff) | ((uint32_t)(CAN_SET_I_VALUE) << 8);
    _update_can_data(buf, 4);
}

void RAD::set_d_value(float D)
{
    uint8_t ind = 0;
    uint8_t buf[4];
    __buffer_append_float32(buf, D, &ind);
    l_can_msg->address = (l_can_msg->address & 0xff) | ((uint32_t)(CAN_SET_D_VALUE) << 8);
    _update_can_data(buf, 4);
}

void RAD::_update_can_data(uint8_t* buf, size_t size)
{
    (l_can_msg->data).assign(buf, buf + size);
}