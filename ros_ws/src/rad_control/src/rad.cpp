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
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                        ((uint32_t)l_can_id) | ((uint32_t)(CAN_ASSIGN_DEVICE_ID) << 8);
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
    _set_null_data(CAN_CALIBRATE_POS);
}

void RAD::cancel_calibration()
{
    _set_null_data(CAN_CANCEL_CALIBRATE_POS);
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
    __buffer_append_uint32(buf, (uint32_t)speed, &ind);
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_STEPPER_SPEED) << 8);
    _update_can_data(buf, 4);
}

void RAD::get_stepper_speed()
{
    _set_null_data(CAN_GET_STEPPER_SPEED);
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
    _set_null_data(CAN_GET_TARGET_ANGLE);
}

void RAD::get_p_value()
{
    _set_null_data(CAN_GET_P_VALUE);
}

void RAD::get_i_value()
{
    _set_null_data(CAN_GET_I_VALUE);
}

void RAD::get_d_value()
{
    _set_null_data(CAN_GET_D_VALUE);
}

void RAD::save_to_eeprom()
{
    _set_null_data(CAN_SAVE_TO_EEPROM);
}

void RAD::reload_from_eeprom()
{
    _set_null_data(CAN_RELOAD_FROM_EEPROM);
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

void RAD::get_odom_interval()
{
    _set_null_data(CAN_GET_ODOM_INTERVAL);
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

void RAD::get_health_interval()
{
    _set_null_data(CAN_GET_HEALTH_INTERVAL);
}

void RAD::set_drvctrl_mres(uint8_t mres)
{
    uint8_t buf[1];
    buf[0] = mres;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_DRVCTRL_MRES) << 8);
    _update_can_data(buf, 1);
}

void RAD::set_drvconf_tst(bool enabled)
{
    uint8_t buf[1];
    buf[0] = enabled;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_DRVCONF_TST) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_drvconf_tst()
{
    _set_null_data(CAN_GET_DRVCONF_TST);
}

void RAD::set_drvconf_slp(uint8_t slope)
{
    uint8_t buf[1];
    buf[0] = slope & 0b111;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_DRVCONF_SLP) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_drvconf_slp()
{
    _set_null_data(CAN_GET_DRVCONF_SLP);
}

void RAD::set_drvconf_s2g(bool enabled)
{
    uint8_t buf[1];
    buf[0] = enabled;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_DRVCONF_S2G) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_drvconf_s2g()
{
    _set_null_data(CAN_GET_DRVCONF_S2G);
}

void RAD::set_drvconf_ts2g(uint8_t delay)
{
    uint8_t buf[1];
    buf[0] = delay & 0b11;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_DRVCONF_TS2G) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_drvconf_ts2g()
{
    _set_null_data(CAN_GET_DRVCONF_TS2G);
}

void RAD::set_drvconf_sdoff(bool disabled)
{
    uint8_t buf[1];
    buf[0] = disabled;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_DRVCONF_SDOFF) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_drvconf_sdoff()
{
    _set_null_data(CAN_GET_DRVCONF_SDOFF);
}

void RAD::set_drvconf_vsense(bool enabled)
{
    uint8_t buf[1];
    buf[0] = enabled;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_DRVCONF_VSENSE) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_drvconf_vsense()
{
    _set_null_data(CAN_GET_DRVCONF_VSENSE);
}

void RAD::set_drvconf_rdsel(uint8_t readout)
{
    uint8_t buf[1];
    buf[0] = readout & 0b11;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_DRVCONF_RDSEL) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_drvconf_rdsel()
{
    _set_null_data(CAN_GET_DRVCONF_RDSEL);
}

void RAD::set_drvconf_otsens(bool lower_shutdown)
{
    uint8_t buf[1];
    buf[0] = lower_shutdown;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_DRVCONF_OTSENS) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_drvconf_otsens()
{
    _set_null_data(CAN_GET_DRVCONF_OTSENS);
}

void RAD::set_drvconf_shrtsens(bool sensitive)
{
    uint8_t buf[1];
    buf[0] = sensitive;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_DRVCONF_SHRTSENS) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_drvconf_shrtsens()
{
    _set_null_data(CAN_GET_DRVCONF_SHRTSENS);
}

void RAD::set_drvconf_en_pfd(bool enabled)
{
    uint8_t buf[1];
    buf[0] = enabled;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_DRVCONF_EN_PFD) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_drvconf_en_pfd()
{
    _set_null_data(CAN_GET_DRVCONF_EN_PFD);
}

void RAD::set_drvconf_en_s2vs(bool enabled)
{
    uint8_t buf[1];
    buf[0] = enabled;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_DRVCONF_EN_S2VS) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_drvconf_en_s2vs()
{
    _set_null_data(CAN_GET_DRVCONF_EN_S2VS);
}

void RAD::get_drvctrl_mres()
{
    _set_null_data(CAN_GET_DRVCTRL_MRES);
}

void RAD::set_drvctrl_dedge(bool enabled)
{
    uint8_t buf[1];
    buf[0] = enabled;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_DRVCTRL_DEDGE) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_drvctrl_dedge()
{
    _set_null_data(CAN_GET_DRVCTRL_DEDGE);
}

void RAD::set_drvctrl_intpol(bool enabled)
{
    uint8_t buf[1];
    buf[0] = enabled;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_DRVCTRL_INTPOL) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_drvctrl_intpol()
{
    _set_null_data(CAN_GET_DRVCTRL_INTPOL);
}

void RAD::set_sgcsconf_cs(uint8_t cs)
{
    uint8_t buf[1];
    buf[0] = cs;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_SGCSCONF_CS) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_sgcsconf_cs()
{
    _set_null_data(CAN_GET_SGCSCONF_CS);
}

void RAD::set_chopconf_tbl(uint8_t interval)
{
    uint8_t buf[1];
    buf[0] = interval & 0b11;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_CHOPCONF_TBL) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_chopconf_tbl()
{
    _set_null_data(CAN_SET_CHOPCONF_TBL);
}

void RAD::set_chopconf_chm(bool mode)
{
    uint8_t buf[1];
    buf[0] = mode;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_CHOPCONF_CHM) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_chopconf_chm()
{
    _set_null_data(CAN_SET_CHOPCONF_CHM);
}

void RAD::set_chopconf_rndtf(bool enabled)
{
    uint8_t buf[1];
    buf[0] = enabled;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_CHOPCONF_RNDTF) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_chopconf_rndtf()
{
    _set_null_data(CAN_GET_CHOPCONF_RNDTF);
}

void RAD::set_chopconf_hdec(uint8_t value)
{
    uint8_t buf[1];
    buf[0] = value & 0b11;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_CHOPCONF_HDEC) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_chopconf_hdec()
{
    _set_null_data(CAN_GET_CHOPCONF_HDEC);
}

void RAD::set_chopconf_hend(uint8_t value)
{
    uint8_t buf[1];
    buf[0] = value & 0b1111;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_CHOPCONF_HEND) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_chopconf_hend()
{
    _set_null_data(CAN_GET_CHOPCONF_HEND);
}

void RAD::set_chopconf_hstrt(uint8_t value)
{
    uint8_t buf[1];
    buf[0] = value & 0b111;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_CHOPCONF_HSTRT) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_chopconf_hstrt()
{
    _set_null_data(CAN_GET_CHOPCONF_HSTRT);
}

void RAD::set_chopconf_toff(uint8_t toff)
{
    uint8_t buf[1];
    buf[0] = toff;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_CHOPCONF_TOFF) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_chopconf_toff()
{
    _set_null_data(CAN_GET_CHOPCONF_TOFF);
}

void RAD::set_smarten_seimin(bool lower_current)
{
    uint8_t buf[1];
    buf[0] = lower_current;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_SMARTEN_SEIMIN) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_smarten_seimin()
{
    _set_null_data(CAN_GET_SMARTEN_SEIMIN);
}

void RAD::set_smarten_sedn(uint8_t samples)
{
    uint8_t buf[1];
    buf[0] = samples & 0b11;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_SMARTEN_SEDN) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_smarten_sedn()
{
    _set_null_data(CAN_GET_SMARTEN_SEDN);
}

void RAD::set_smarten_seup(uint8_t samples)
{
    uint8_t buf[1];
    buf[0] = samples & 0b11;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_SMARTEN_SEUP) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_smarten_seup()
{
    _set_null_data(CAN_GET_SMARTEN_SEUP);
}

void RAD::set_smarten_semin(uint8_t threshold)
{
    uint8_t buf[1];
    buf[0] = threshold & 0b1111;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_SMARTEN_SEMIN) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_smarten_semin()
{
    _set_null_data(CAN_GET_SMARTEN_SEMIN);
}

void RAD::set_smarten_semax(uint8_t threshold)
{
    uint8_t buf[1];
    buf[0] = threshold & 0b1111;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_SMARTEN_SEMAX) << 8);
    _update_can_data(buf, 1);
}

void RAD::get_smarten_semax()
{
    _set_null_data(CAN_GET_SMARTEN_SEMAX);
}



void RAD::set_min_output(uint16_t min_output)
{
    uint8_t buf[2];
    buf[0] = ((min_output & 0xff00) >> 8);
    buf[1] = (min_output & 0xff);
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                        ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_PID_MIN_OUTPUT) << 8);
    _update_can_data(buf, 2);
}

void RAD::get_min_output()
{
    _set_null_data(CAN_GET_PID_MIN_OUTPUT);
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
    _set_null_data(CAN_GET_PID_MAX_OUTPUT);
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

void RAD::set_home_position(uint32_t pos)
{
    uint8_t buf[4];
    uint8_t i = 0;
    __buffer_append_uint32(buf, pos, &i);
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                    ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_HOME_POSITION) << 8);
    _update_can_data(buf, 4);
}

void RAD::get_home_position()
{
    _set_null_data(CAN_GET_HOME_POSITION);
}

void RAD::set_home_offset()
{
    _set_null_data(CAN_SET_HOME_OFFSET);
}

void RAD::get_home_offset()
{
    _set_null_data(CAN_GET_HOME_OFFSET);
}

void RAD::_set_null_data(RAD_CAN_MSG msg)
{
    uint8_t buf[1];
    buf[0] = 0;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_RAD << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                        ((uint32_t)l_can_id) | ((uint32_t)(msg) << 8);
    _update_can_data(buf, 1);
}

void RAD::_update_can_data(uint8_t* buf, size_t size)
{
    (l_can_msg->data).assign(buf, buf + size);
}