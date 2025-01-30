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
        case CAN_SEND_CARD_INPUT_FAULT:
            status->card_input_fault = (bool)buf[0];
            break;
        case CAN_SEND_CARD_OUTPUT_A_FAULT:
            status->card_output_a_fault = (bool)buf[0];
            break;
        case CAN_SEND_CARD_OUTPUT_B_FAULT:
            status->card_output_b_fault = (bool)buf[0];
            break;
        case CAN_SEND_CARD_TEMPERATURE:
            status->card_temperature = __buffer_get_float64(buf, &i);
            break;
        case CAN_SEND_CARD_INPUT_CURRENT:
            status->card_input_current = __buffer_get_float64(buf, &i);
            break;
        case CAN_SEND_CARD_OUTPUT_DIAGNOSTIC_A:
            status->card_output_diagnostic_a = __buffer_get_uint16(buf, &i);
            break;
        case CAN_SEND_CARD_OUTPUT_POWER_A:
            status->card_output_power_a = __buffer_get_float64(buf, &i);
            break;
        case CAN_SEND_CARD_OUTPUT_CURRENT_A:
            status->card_output_current_a = __buffer_get_float64(buf, &i);
            break;
        case CAN_SEND_CARD_OUTPUT_VOLTAGE_A:
            status->card_output_voltage_a = __buffer_get_float64(buf, &i);
            break;
        case CAN_SEND_CARD_OUTPUT_DIAGNOSTIC_B:
            status->card_output_diagnostic_b = __buffer_get_uint16(buf, &i);
            break;
        case CAN_SEND_CARD_OUTPUT_POWER_B:
            status->card_output_power_b = __buffer_get_float64(buf, &i);
            break;
        case CAN_SEND_CARD_OUTPUT_CURRENT_B:
            status->card_output_current_b = __buffer_get_float64(buf, &i);
            break;
        case CAN_SEND_CARD_OUTPUT_VOLTAGE_B:
            status->card_output_voltage_b = __buffer_get_float64(buf, &i);
            break;
        case CAN_SEND_HEALTH_STATUS:
            status->eeprom_status = buf[0];
            status->mux_status = buf[1];
            status->card_status[0] = buf[2]; // assuming card_status is an array, change if necessary
            status->card_status[1] = buf[3];
            status->card_status[2] = buf[4];
            status->card_status[3] = buf[5];
            break;
        default:
            return 0; // Unknown message type
    }
    return 1; 
}

void VIPER::set_can_id(uint8_t can_id)
{
    uint8_t buf[1];
    buf[0] = can_id;
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id); // No equivalent to CAN_ASSIGN_DEVICE_ID in VIPER
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

// setter functions

void VIPER::set_health_interval(uint32_t health_interval)
{
    uint8_t ind = 0;
    uint8_t buf[4]; // Corrected buffer size
    __buffer_append_uint32(buf, health_interval, &ind); // Fixed variable name
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_HEALTH_INTERVAL) << 8);
    _update_can_data(buf, 4); // Buffer size updated to 4 bytes
}

void VIPER::set_card_interval(uint32_t card_interval)
{
    uint8_t ind = 0;
    uint8_t buf[4]; // Corrected buffer size
    __buffer_append_uint32(buf, card_interval, &ind); // Fixed variable name
    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_CARD_INTERVAL) << 8); // Fixed incorrect command ID
    _update_can_data(buf, 4); // Buffer size updated to 4 bytes
}

void VIPER::set_mux_value(uint8_t mux_value)
{
    uint8_t buf[1];
    buf[0] = mux_value; // Store the MUX value in the buffer

    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_SET_MUX_VALUE) << 8);

    _update_can_data(buf, 1); // Send the single-byte data
}

// getter functions

void VIPER::get_card_interval()
{
    uint8_t buf[1];
    buf[0] = 0; // No extra data needed

    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_GET_CARD_INTERVAL) << 8);

    _update_can_data(buf, 1);
}

void VIPER::get_all_card_data()
{
    uint8_t buf[1];
    buf[0] = 0; // No extra data needed

    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_GET_ALL_CARD_DATA) << 8);

    _update_can_data(buf, 1);
}

void VIPER::get_card_data(uint8_t target)
{
    uint8_t buf[1];
    buf[0] = target; // Specify the target card

    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_GET_CARD_DATA) << 8);

    _update_can_data(buf, 1);
}

// enable and disable functions

void VIPER::disable_card(uint8_t card)
{
    uint8_t buf[1];
    buf[0] = card; // Specify the card to disable

    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_DISABLE_CARD) << 8);

    _update_can_data(buf, 1);
}

void VIPER::disable_all_cards()
{
    uint8_t buf[1];
    buf[0] = 0; // No extra data needed

    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_DISABLE_ALL_CARDS) << 8);

    _update_can_data(buf, 1);
}

void VIPER::enable_card(uint8_t card_id)
{
    uint8_t buf[1];
    buf[0] = card_id; // Specify the card to enable

    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_ENABLE_CARD) << 8);

    _update_can_data(buf, 1);
}

void VIPER::enable_all_cards()
{
    uint8_t buf[1];
    buf[0] = 0; // No extra data needed

    l_can_msg->address = (CAN_MESSAGE_IDENTIFIER_VIPER << CAN_MESSAGE_IDENTIFIER_OFFSET) | 
                         ((uint32_t)l_can_id) | ((uint32_t)(CAN_ENABLE_ALL_CARDS) << 8);

    _update_can_data(buf, 1);
}







