#pragma once
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/ca_nraw.hpp"
#include "custom_interfaces/msg/rad_status.hpp"

//#define BIG_ENDIANNESS


using namespace custom_interfaces::msg;

typedef enum RAD_CAN_MSG : uint8_t
{
    CAN_SET_TARGET_ANGLE      = 0x01,
    CAN_SET_STEPPER_SPEED     = 0x03,

    CAN_SET_P_VALUE           = 0x05,
    CAN_SET_I_VALUE           = 0x07,
    CAN_SET_D_VALUE           = 0x09,

    CAN_SET_DRVCTRL_REGISTER  = 0x11,
    CAN_SET_CHOPCONF_REGISTER = 0x13,
    CAN_SET_SMARTEN_REGISTER  = 0x15,
    CAN_SET_SGSCONF_REGISTER  = 0x17,
    CAN_SET_DRVCONF_REGISTER  = 0x19,

    CAN_SET_RAD_TYPE          = 0x21,
    CAN_CAPTURE_HOME_POSITION = 0x23,

    CAN_SET_ADC1_LOW          = 0x25,
    CAN_SET_ADC1_HIGH         = 0x27,
    CAN_SET_ADC2_LOW          = 0x29,
    CAN_SET_ADC2_HIGH         = 0x31,

    CAN_SET_RAD_FLAGS         = 0x33,
    CAN_SET_WATCHDOG_INTERVAL = 0x35,
    CAN_SAVE_TO_EEPROM        = 0x37,

// ------------------------------------
    CAN_GET_TARGET_ANGLE      = 0x02,
    CAN_GET_STEPPER_SPEED     = 0x04,

    CAN_GET_P_VALUE           = 0x06,
    CAN_GET_I_VALUE           = 0x08,
    CAN_GET_D_VALUE           = 0x10,

    CAN_GET_DRVCTRL_REGISTER  = 0x12,
    CAN_GET_CHOPCONF_REGISTER = 0x14,
    CAN_GET_SMARTEN_REGISTER  = 0x16,
    CAN_GET_SGSCONF_REGISTER  = 0x18,
    CAN_GET_DRVCONF_REGISTER  = 0x20,

    CAN_GET_RAD_TYPE          = 0x22,
    CAN_RESET_HOME_POSITION   = 0x24,

    CAN_GET_ADC1_LOW          = 0x26,
    CAN_GET_ADC1_HIGH         = 0x28,
    CAN_GET_ADC2_LOW          = 0x30,
    CAN_GET_ADC2_HIGH         = 0x32,

    CAN_GET_RAD_FLAGS         = 0x34,
    CAN_GET_WATCHDOG_INTERVAL = 0x36

} RAD_CAN_MSG;


typedef union __can_data
{
    double angle;

    uint64_t speed;

    uint32_t P_val, 
             I_val, 
             D_val, 
             reg_val, 
             flags;

    uint16_t adc;

    uint8_t arr[8];

} CAN_DATA;


class RAD
{
public:
    RAD(CANraw* can_msg);
    RAD(CANraw* can_msg, uint32_t can_id);

    void set_target_angle(double angle);
    void set_stepper_speed(uint64_t speed);
    void set_p_value(uint32_t P);
    void set_i_value(uint32_t I);
    void set_d_value(uint32_t D);

private:
    void _update_can_data(void);

    CANraw* l_can_msg;
    CAN_DATA l_can_data;
};

uint8_t decode_can_msg(CANraw* can_msg, RadStatus* status);
