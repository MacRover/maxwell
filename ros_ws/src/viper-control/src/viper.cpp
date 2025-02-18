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

uint8_t decode_can_msg(const CANraw* can_msg, ViperCardStatus* status, ViperStatus* state)
{
    uint8_t* buf = (uint8_t*) &(can_msg->data[0]);
    uint8_t i = 0;

    switch(((can_msg->address) >> 8) & 0xff)
    {
        case CAN_SEND_CARD_INPUT_VOLTAGE:
            status->input_voltage = __buffer_get_float64(buf, &i);
            break;
        case CAN_SEND_CARD_INPUT_FAULT:
            status->input_fault = (bool)buf[0];
            break;
        case CAN_SEND_CARD_OUTPUT_A_FAULT:
            status->output_a_fault = (bool)buf[0];
            break;
        case CAN_SEND_CARD_OUTPUT_B_FAULT:
            status->output_b_fault = (bool)buf[0];
            break;
        case CAN_SEND_CARD_TEMPERATURE_BACKPLANE:
            status->temperature_backplane = __buffer_get_float64(buf, &i);
            break;
        case CAN_SEND_CARD_TEMPERATURE_CARD_A:
            status->temperature_card_a = __buffer_get_float64(buf, &i);
            break;
        case CAN_SEND_CARD_TEMPERATURE_CARD_B:
            status->temperature_card_b = __buffer_get_float64(buf, &i);
            break;
        case CAN_SEND_CARD_INPUT_CURRENT:
            status->input_current = __buffer_get_float64(buf, &i);
            break;
        case CAN_SEND_CARD_OUTPUT_POWER_A:
            status->output_power_a = __buffer_get_float64(buf, &i);
            break;
        case CAN_SEND_CARD_OUTPUT_CURRENT_A:
            status->output_current_a = __buffer_get_float64(buf, &i);
            break;
        case CAN_SEND_CARD_OUTPUT_VOLTAGE_A:
            status->output_voltage_a = __buffer_get_float64(buf, &i);
            break;
        case CAN_SEND_CARD_OUTPUT_POWER_B:
            status->output_power_b = __buffer_get_float64(buf, &i);
            break;
        case CAN_SEND_CARD_OUTPUT_CURRENT_B:
            status->output_current_b = __buffer_get_float64(buf, &i);
            break;
        case CAN_SEND_CARD_OUTPUT_VOLTAGE_B:
            status->output_voltage_b = __buffer_get_float64(buf, &i);
            break;
        case CAN_SEND_HEALTH_STATUS:
            state->eeprom_status = buf[0];
            state->mux_status = buf[1];
            state->card_0_status = buf[2]; 
            state->card_1_status = buf[3];
            state->card_2_status = buf[4];
            state->card_3_status = buf[5];
            break;
        default:
            return 0; // Unknown message type
    }
    return 1; 
}

void VIPER::set_can_id(uint8_t can_id)
{
    uint8_t buf[1] = { can_id };

    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((can_id & CAN_MESSAGE_DEVICE_ID_MASK) << CAN_MESSAGE_DEVICE_ID_OFFSET); 

    _update_can_data(buf, 1);
    this->l_can_id = can_id;
}

void VIPER::save_to_eeprom()
{
    uint8_t buf[1] = { 0 };

    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((l_can_id & CAN_MESSAGE_DEVICE_ID_MASK) << CAN_MESSAGE_DEVICE_ID_OFFSET) |
                         ((uint32_t)(CAN_SAVE_TO_EEPROM) << 8); 

    _update_can_data(buf, 1);
}

// setter functions


void VIPER::set_health_interval(uint32_t health_interval)
{
    uint8_t ind = 0;
    uint8_t buf[4];
    __buffer_append_uint32(buf, health_interval, &ind);

    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) |
                         ((l_can_id & CAN_MESSAGE_DEVICE_ID_MASK) << CAN_MESSAGE_DEVICE_ID_OFFSET) |
                         ((0 & CAN_MESSAGE_CARD_ID_MASK) << CAN_MESSAGE_CARD_ID_OFFSET) | 
                         ((CAN_SET_HEALTH_INTERVAL) << 8);

    _update_can_data(buf, 4);
}

void VIPER::set_card_interval(uint32_t card_interval)
{
    uint8_t ind = 0;
    uint8_t buf[4];
    __buffer_append_uint32(buf, card_interval, &ind);

    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) |
                         ((l_can_id & CAN_MESSAGE_DEVICE_ID_MASK) << CAN_MESSAGE_DEVICE_ID_OFFSET) |
                         ((0 & CAN_MESSAGE_CARD_ID_MASK) << CAN_MESSAGE_CARD_ID_OFFSET) | 
                         ((CAN_SET_CARD_INTERVAL) << 8);

    _update_can_data(buf, 4);
}

void VIPER::set_mux_value(uint8_t mux_value)
{
    uint8_t buf[1];
    buf[0] = mux_value;

    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) |
                         ((l_can_id & CAN_MESSAGE_DEVICE_ID_MASK) << CAN_MESSAGE_DEVICE_ID_OFFSET) |
                         ((0 & CAN_MESSAGE_CARD_ID_MASK) << CAN_MESSAGE_CARD_ID_OFFSET) | 
                         ((CAN_SET_MUX_VALUE) << 8);

    _update_can_data(buf, 1);
}

// getter functions

void VIPER::get_card_interval()
{
    uint8_t buf[1];
    buf[0] = 0; 

    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((l_can_id & CAN_MESSAGE_DEVICE_ID_MASK) << CAN_MESSAGE_DEVICE_ID_OFFSET) | 
                         ((uint32_t)(CAN_GET_CARD_INTERVAL) << 8); 

    _update_can_data(buf, 1);
}

void VIPER::get_all_card_data()
{
    uint8_t buf[1];
    buf[0] = 0; 

    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((l_can_id & CAN_MESSAGE_DEVICE_ID_MASK) << CAN_MESSAGE_DEVICE_ID_OFFSET) | 
                         ((uint32_t)(CAN_GET_ALL_CARD_DATA) << 8); 

    _update_can_data(buf, 1);
}

void VIPER::get_card_data(uint8_t target)
{
    uint8_t buf[1];
    buf[0] = target;

    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) |
                         ((l_can_id & CAN_MESSAGE_DEVICE_ID_MASK) << CAN_MESSAGE_DEVICE_ID_OFFSET) |
                         ((target & CAN_MESSAGE_CARD_ID_MASK) << CAN_MESSAGE_CARD_ID_OFFSET) |
                         ((CAN_GET_CARD_DATA) << 8);

    _update_can_data(buf, 1);
}

// enable and disable functions

void VIPER::disable_card(uint8_t card)
{
    uint8_t buf[1];
    buf[0] = card;

    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) |
                         ((l_can_id & CAN_MESSAGE_DEVICE_ID_MASK) << CAN_MESSAGE_DEVICE_ID_OFFSET) |
                         ((card & CAN_MESSAGE_CARD_ID_MASK) << CAN_MESSAGE_CARD_ID_OFFSET) |
                         ((CAN_DISABLE_CARD) << 8);

    _update_can_data(buf, 1);
}

void VIPER::disable_all_cards()
{
    uint8_t buf[1];
    buf[0] = 0; 

    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((l_can_id & CAN_MESSAGE_DEVICE_ID_MASK) << CAN_MESSAGE_DEVICE_ID_OFFSET) | 
                         ((uint32_t)(CAN_DISABLE_ALL_CARDS) << 8);

    _update_can_data(buf, 1);
}

void VIPER::enable_all_cards()
{
    uint8_t buf[1];
    buf[0] = 0; 

    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((l_can_id & CAN_MESSAGE_DEVICE_ID_MASK) << CAN_MESSAGE_DEVICE_ID_OFFSET) | 
                         ((uint32_t)(CAN_ENABLE_ALL_CARDS) << 8);

    _update_can_data(buf, 1);
}

void VIPER::enable_card(uint8_t card_id)
{
    uint8_t buf[1];
    buf[0] = card_id;

    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) |
                         ((l_can_id & CAN_MESSAGE_DEVICE_ID_MASK) << CAN_MESSAGE_DEVICE_ID_OFFSET) |
                         ((card_id & CAN_MESSAGE_CARD_ID_MASK) << CAN_MESSAGE_CARD_ID_OFFSET) |
                         ((CAN_ENABLE_CARD) << 8);

    _update_can_data(buf, 1);
}

void VIPER::_update_can_data(uint8_t* buf, size_t size)
{
    (l_can_msg->data).assign(buf, buf + size);
}
