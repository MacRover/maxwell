#pragma once
#include <vector>
#include <byteswap.h>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/ca_nraw.hpp"
#include "custom_interfaces/msg/rad_status.hpp"

#define BIG_ENDIANNESS


using namespace custom_interfaces::msg;

typedef enum RAD_CAN_MSG : uint8_t
{
    CAN_SET_TARGET_ANGLE      = 0x01,
    CAN_SET_STEPPER_SPEED     = 0x03,

    CAN_SET_P_VALUE           = 0x05,
    CAN_SET_I_VALUE           = 0x07,
    CAN_SET_D_VALUE           = 0x08,

    // CAN_SET_DRVCTRL_REGISTER  = 0x11,
    // CAN_SET_CHOPCONF_REGISTER = 0x13,
    // CAN_SET_SMARTEN_REGISTER  = 0x15,
    // CAN_SET_SGSCONF_REGISTER  = 0x17,
    // CAN_SET_DRVCONF_REGISTER  = 0x19,

    // CAN_SET_RAD_TYPE          = 0x21,
    // CAN_CAPTURE_HOME_POSITION = 0x23,

    // CAN_SET_ADC1_LOW          = 0x25,
    // CAN_SET_ADC1_HIGH         = 0x27,
    // CAN_SET_ADC2_LOW          = 0x29,
    // CAN_SET_ADC2_HIGH         = 0x31,

    // CAN_SET_RAD_FLAGS         = 0x33,
    // CAN_SET_WATCHDOG_INTERVAL = 0x35,
    // CAN_SAVE_TO_EEPROM        = 0x37,

// ------------------------------------
    // CAN_GET_TARGET_ANGLE      = 0x02,
    // CAN_GET_STEPPER_SPEED     = 0x04,

    // CAN_GET_P_VALUE           = 0x06,
    // CAN_GET_I_VALUE           = 0x08,
    // CAN_GET_D_VALUE           = 0x10,

    // CAN_GET_DRVCTRL_REGISTER  = 0x12,
    // CAN_GET_CHOPCONF_REGISTER = 0x14,
    // CAN_GET_SMARTEN_REGISTER  = 0x16,
    // CAN_GET_SGSCONF_REGISTER  = 0x18,
    // CAN_GET_DRVCONF_REGISTER  = 0x20,

    // CAN_GET_RAD_TYPE          = 0x22,
    // CAN_RESET_HOME_POSITION   = 0x24,

    // CAN_GET_ADC1_LOW          = 0x26,
    // CAN_GET_ADC1_HIGH         = 0x28,
    // CAN_GET_ADC2_LOW          = 0x30,
    // CAN_GET_ADC2_HIGH         = 0x32,

    // CAN_GET_RAD_FLAGS         = 0x34,
    // CAN_GET_WATCHDOG_INTERVAL = 0x36

    CAN_STATUS_1                 = 0x09,
    CAN_STATUS_2                 = 0x14,
    CAN_STATUS_3                 = 0x15

} RAD_CAN_MSG;


class RAD
{
public:
    RAD(CANraw* can_msg);
    RAD(CANraw* can_msg, uint8_t can_id);

    void set_target_angle(float angle);
    void set_stepper_speed(float speed);
    void set_p_value(float P);
    void set_i_value(float I);
    void set_d_value(float D);

private:
    void _update_can_data(uint8_t* buf, size_t size);

    CANraw* l_can_msg;
};

float __buffer_get_float32(uint8_t* buf, uint8_t* ind);
void __buffer_append_float32(uint8_t* buf, float n, uint8_t* ind);
uint8_t decode_can_msg(CANraw* can_msg, RadStatus* status);
