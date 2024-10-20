#pragma once
#include <vector>
#include <byteswap.h>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/ca_nraw.hpp"
#include "custom_interfaces/msg/rad_status.hpp"

#define BIG_ENDIANNESS


using namespace custom_interfaces::msg;

typedef enum GLOBAL_CAN_MSG : uint8_t
{
    ESTOP_MESSAGE = 0x31,
    DISABLE_MESSAGE = 0x00,
    ENABLE_MESSAGE = 0x02,
    HEALTH_STATUS_PING = 0x03,
    ASSIGN_DEVICE_ID = 0x04
} GLOBAL_CAN_MSG;

typedef enum RAD_CAN_MSG : uint8_t
{
    SET_TARGET_ANGLE = 0x01,
    GET_ENCODER_VALUE = 0x02,
    SET_STEPPER_SPEED = 0x03,
    GET_STEPPER_SPEED = 0x04,
    SET_P_VALUE = 0x05,
    GET_P_VALUE = 0x06,
    SET_I_VALUE = 0x07,
    GET_I_VALUE = 0x08,
    SET_D_VALUE = 0x09,
    GET_D_VALUE = 0x0A,
    SET_RAD_TYPE = 0x0B,
    GET_RAD_TYPE = 0x0C,
    SET_HOME_POSITION = 0x0D,
    GET_HOME_POSITION = 0x0E,
    SET_ODOM_INTERVAL = 0x0F,
    GET_ODOM_INTERVAL = 0x10,
    SAVE_TO_EEPROM = 0x11,
    RELOAD_FROM_EEPROM = 0x12,
    SET_HEALTH_INTERVAL = 0x13,
    GET_HEALTH_INTERVAL = 0x14,
    START_CALIBRATE = 0x15,
    CANCEL_CALIBRATION = 0x16,
    SET_DRVCONF_TST = 0x17,
    GET_DRVCONF_TST = 0x18,
    SET_DRVCONF_SLP = 0x19,
    GET_DRVCONF_SLP = 0x1A,
    SET_DRVCONF_DIS_S2G = 0x1B,
    GET_DRVCONF_DIS_S2G = 0x1C,
    SET_DRVCONF_TS2G = 0x1D,
    GET_DRVCONF_TS2G = 0x1E,
    SET_DRVCONF_SDOFF = 0x1F,
    GET_DRVCONF_SDOFF = 0x20,
    SET_DRVCONF_VSENSE = 0x21,
    GET_DRVCONF_VSENSE = 0x22,
    SET_DRVCONF_RDSEL = 0x23,
    GET_DRVCONF_RDSEL = 0x24,
    SET_DRVCONF_OTSENS = 0x25,
    GET_DRVCONF_OTSENS = 0x26,
    SET_DRVCONF_SHRTSENS = 0x27,
    GET_DRVCONF_SHRTSENS = 0x28,
    SET_DRVCONF_EN_PFD = 0x29,
    GET_DRVCONF_EN_PFD = 0x2A,
    SET_DRVCONF_EN_S2VS = 0x2B,
    GET_DRVCONF_EN_S2VS = 0x2C,
    SET_SGCSCONF_SFILT = 0x2D,
    GET_SGCSCONF_SFILT = 0x2E,
    SET_SGCSCONF_SGT = 0x2F,
    GET_SGCSCONF_SGT = 0x30,
    SET_SGCSCONF_CS = 0x31,
    GET_SGCSCONF_CS = 0x32,
    SET_SMARTEN_SEIMIN = 0x33,
    GET_SMARTEN_SEIMIN = 0x34,
    SET_SMARTEN_SEDN = 0x35,
    GET_SMARTEN_SEDN = 0x36,
    SET_SMARTEN_SEMAX = 0x37,
    GET_SMARTEN_SEMAX = 0x38,
    SET_SMARTEN_SEUP = 0x39,
    GET_SMARTEN_SEUP = 0x3A,
    SET_SMARTEN_SEMIN = 0x3B,
    GET_SMARTEN_SEMIN = 0x3C,
    SET_CHOPCONF_TBL = 0x3D,
    GET_CHOPCONF_TBL = 0x3E,
    SET_CHOPCONF_CHM = 0x3F,
    GET_CHOPCONF_CHM = 0x40,
    SET_CHOPCONF_RNDTF = 0x41,
    GET_CHOPCONF_RNDTF = 0x42,
    SET_CHOPCONF_HDEC = 0x43,
    GET_CHOPCONF_HDEC = 0x44,
    SET_CHOPCONF_HEND = 0x45,
    GET_CHOPCONF_HEND = 0x46,
    SET_CHOPCONF_HSTRT = 0x47,
    GET_CHOPCONF_HSTRT = 0x48,
    SET_CHOPCONF_TOFF = 0x49,
    GET_CHOPCONF_TOFF = 0x4A,
    SET_DRVCTRL_INTPOL = 0x4B,
    GET_DRVCTRL_INTPOL = 0x4C,
    SET_DRVCTRL_DEDGE = 0x4D,
    GET_DRVCTRL_DEDGE = 0x4E,
    SET_DRVCTRL_MRES = 0x4F,
    GET_DRVCTRL_MRES = 0x50,
    PULSE_STEPPER = 0x51,

    
    SEND_ODOM_ANGLE = 0xFB,
    SEND_HEALTH_STATUS = 0xFC

} RAD_CAN_MSG;


typedef enum __rad_can_id : uint8_t
{
    RAD__DRIVE__FRONT_RIGHT = 0x01,
    RAD__DRIVE__BACK_RIGHT = 0x02,
    RAD__DRIVE__BACK_LEFT = 0x03,
    RAD__DRIVE__FRONT_LEFT = 0x04

} RAD_ID;


class RAD
{
public:
    RAD(CANraw* can_msg);
    RAD(CANraw* can_msg, RAD_ID can_id);

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
uint8_t decode_can_msg(const CANraw* can_msg, RadStatus* status);
