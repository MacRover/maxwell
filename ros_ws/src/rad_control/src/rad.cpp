#include "rad_control/rad.hpp"


RAD::RAD(CANraw* can_msg) : l_can_msg(can_msg), l_offset(0.0), l_factor(1.0) { }

RAD::RAD(CANraw* can_msg, uint8_t can_id) : l_can_msg(can_msg), l_offset(0.0), l_factor(1.0)
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
            status->ls_state = (bool)buf[4];
            break;
        default:
            return 0;
    }
    return 1;
}

void RAD::set_can_id(uint8_t can_id)
{
    uint8_t buf[1];
    buf[0] = can_id;
    l_can_msg->address = ((uint32_t)l_can_id) | ((uint32_t)(CAN_ASSIGN_DEVICE_ID) << 8);
    _update_can_data(buf, 1);
    // Set RAD ID after sending command
    this->l_can_id = can_id;
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
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_CALIBRATE_POS) << 8);
    _update_can_data(buf, 1);
}

void RAD::cancel_calibration()
{
    uint8_t buf[1];
    buf[0] = 0;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                        ((uint32_t)l_can_id) | ((uint32_t)(CAN_CANCEL_CALIBRATE_POS) << 8);
    _update_can_data(buf, 1);
}

void RAD::set_rad_type(uint8_t type)
{
    uint8_t buf[1];
    buf[0] = type;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_RAD_TYPE) << 8);
    _update_can_data(buf, 1);
}

void RAD::set_target_angle(double angle)
{
    uint8_t ind = 0;
    uint8_t buf[8];
    __buffer_append_float64(buf, l_factor*(angle + l_offset), &ind);
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_TARGET_ANGLE) << 8);
    _update_can_data(buf, 8);
}

void RAD::set_stepper_speed(float speed)
{
    uint8_t ind = 0;
    uint8_t buf[4];
    __buffer_append_uint32(buf, speed, &ind);
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_STEPPER_SPEED) << 8);
    _update_can_data(buf, 4);
}

void RAD::set_p_value(double P)
{
    uint8_t ind = 0;
    uint8_t buf[8];
    __buffer_append_float64(buf, P, &ind);
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_P_VALUE) << 8);
    _update_can_data(buf, 8);
}

void RAD::set_i_value(double I)
{
    uint8_t ind = 0;
    uint8_t buf[8];
    __buffer_append_float64(buf, I, &ind);
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_I_VALUE) << 8);
    _update_can_data(buf, 8);
}

void RAD::set_d_value(double D)
{
    uint8_t ind = 0;
    uint8_t buf[8];
    __buffer_append_float64(buf, D, &ind);
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_D_VALUE) << 8);
    _update_can_data(buf, 8);
}

void RAD::get_target_angle()
{
    uint8_t buf[1];
    buf[0] = 0;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_GET_TARGET_ANGLE) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_p_value()
{
    uint8_t buf[1];
    buf[0] = 0;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_GET_P_VALUE) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_i_value()
{
    uint8_t buf[1];
    buf[0] = 0;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_GET_I_VALUE) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_d_value()
{
    uint8_t buf[1];
    buf[0] = 0;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_GET_D_VALUE) << 8);
    _update_can_data(buf, 1);
}

void RAD::save_to_eeprom()
{
    uint8_t buf[1];
    buf[0] = 0;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SAVE_TO_EEPROM) << 8);
    _update_can_data(buf, 1);
}

void RAD::set_odom_interval(uint32_t period)
{
    uint8_t ind = 0;
    uint8_t buf[4];
    __buffer_append_uint32(buf, period, &ind);
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_ODOM_INTERVAL) << 8);
    _update_can_data(buf, 4);
}

void RAD::set_health_interval(uint32_t period)
{
    uint8_t ind = 0;
    uint8_t buf[4];
    __buffer_append_uint32(buf, period, &ind);
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_HEALTH_INTERVAL) << 8);
    _update_can_data(buf, 4);
}

void RAD::set_drvctrl_mres(uint8_t mres)
{
    uint8_t buf[1];
    buf[0] = mres;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_DRVCTRL_MRES) << 8);
    _update_can_data(buf, 1);
}

void RAD::set_sgcsconf_cs(uint8_t cs)
{
    uint8_t buf[1];
    buf[0] = cs;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_SGCSCONF_CS) << 8);
    _update_can_data(buf, 1);
}

void RAD::set_error_thres(uint16_t thres)
{
    uint8_t buf[2];
    buf[0] = ((thres & 0xff00) >> 8);
    buf[1] = (thres & 0xff);
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                        ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_PID_ERROR_THRESHOLD) << 8);
    _update_can_data(buf, 2);
}

void RAD::get_error_thres()
{
    uint8_t buf[1];
    buf[0] = 0;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                        ((uint32_t)l_can_id) | ((uint32_t)(CAN_GET_PID_ERROR_THRESHOLD) << 8);
    _update_can_data(buf, 1);
}

void RAD::set_max_output(uint16_t max_output)
{
    uint8_t buf[2];
    buf[0] = ((max_output & 0xff00) >> 8);
    buf[1] = (max_output & 0xff);
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                        ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_PID_MAX_OUTPUT) << 8);
    _update_can_data(buf, 2);
}

void RAD::get_max_output()
{
    uint8_t buf[1];
    buf[0] = 0;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                        ((uint32_t)l_can_id) | ((uint32_t)(CAN_GET_PID_MAX_OUTPUT) << 8);
    _update_can_data(buf, 1);
}


void RAD::pulse_stepper(float steps)
{
    uint8_t buf[4];
    uint8_t i = 0;
    __buffer_append_float32(buf, steps, &i);
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                    ((uint32_t)l_can_id) | ((uint32_t)(CAN_PULSE_STEPPER) << 8);
    _update_can_data(buf, 4);
}


void RAD::_update_can_data(uint8_t* buf, size_t size)
{
    (l_can_msg->data).assign(buf, buf + size);
}