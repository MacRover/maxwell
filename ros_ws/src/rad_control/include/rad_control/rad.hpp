#pragma once
#include <vector>
#include <byteswap.h>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/ca_nraw.hpp"
#include "custom_interfaces/msg/rad_status.hpp"

#define BIG_ENDIANNESS
#define CAN_MESSAGE_IDENTIFIER_RAD 0x02
#define CAN_MESSAGE_IDENTIFIER_OFFSET 25


using namespace custom_interfaces::msg;

typedef enum RAD_CAN_MSG : uint8_t
{
    CAN_SET_TARGET_ANGLE      = 1,
    CAN_SET_STEPPER_SPEED     = 3,
    
    CAN_SET_P_VALUE           = 5,
    CAN_SET_I_VALUE           = 7,
    CAN_SET_D_VALUE           = 9,
    CAN_SET_RAD_TYPE          = 11,
    CAN_SET_HOME_POSITION     = 13,
    CAN_SET_ODOM_INTERVAL     = 15,
    CAN_SAVE_TO_EEPROM        = 17,
    CAN_SET_HEALTH_INTERVAL   = 19,
    CAN_CALIBRATE_POS         = 21,
    CAN_SET_DRVCONF_TST       = 23,
    CAN_SET_DRVCONF_SLP       = 25,
    CAN_SET_DRVCONF_S2G       = 27,
    CAN_SET_DRVCONF_TS2G      = 29,
    CAN_SET_DRVCONF_SDOFF     = 31,
    CAN_SET_DRVCONF_VSENSE    = 33,
    CAN_SET_DRVCONF_RDSEL     = 35,
    CAN_SET_DRVCONF_OTSENS    = 37,
    CAN_SET_DRVCONF_SHRTSENS  = 39,
    CAN_SET_DRVCONF_EN_PFD    = 41,
    CAN_SET_DRVCONF_EN_S2VS   = 43,
    CAN_SET_SGCSCONF_SFILT    = 45,
    CAN_SET_SGCSCONF_SGT      = 47,
    CAN_SET_SGCSCONF_CS       = 49,
    CAN_SET_SMARTEN_SEIMIN    = 51,
    CAN_SET_SMARTEN_SEDN      = 53,
    CAN_SET_SMARTEN_SEMAX     = 55,
    CAN_SET_SMARTEN_SEUP      = 57,
    CAN_SET_SMARTEN_SEMIN     = 59,
    CAN_SET_CHOPCONF_TBL      = 61,
    CAN_SET_CHOPCONF_CHM      = 63,
    CAN_SET_CHOPCONF_RNDTF    = 65,
    CAN_SET_CHOPCONF_HDEC     = 67,
    CAN_SET_CHOPCONF_HEND     = 69,
    CAN_SET_CHOPCONF_HSTRT    = 71,
    CAN_SET_CHOPCONF_TOFF     = 73,
    CAN_SET_DRVCTRL_INTPOL    = 75,
    CAN_SET_DRVCTRL_DEDGE     = 77,
    CAN_SET_DRVCTRL_MRES      = 79,

// ------------------------------------
    CAN_GET_TARGET_ANGLE      = 2,
    CAN_GET_STEPPER_SPEED     = 4,
    
    CAN_GET_P_VALUE           = 6,
    CAN_GET_I_VALUE           = 8,
    CAN_GET_D_VALUE           = 10,
    CAN_GET_RAD_TYPE          = 12,
    CAN_GET_HOME_POSITION     = 14,
    CAN_GET_ODOM_INTERVAL     = 16,
    CAN_RELOAD_FROM_EEPROM    = 18,
    CAN_GET_HEALTH_INTERVAL   = 20,
    CAN_CANCEL_CALIBRATE_POS  = 22,
    CAN_GET_DRVCONF_TST       = 24,
    CAN_GET_DRVCONF_SLP       = 26,
    CAN_GET_DRVCONF_S2G       = 28,
    CAN_GET_DRVCONF_TS2G      = 30,
    CAN_GET_DRVCONF_SDOFF     = 32,
    CAN_GET_DRVCONF_VSENSE    = 34,
    CAN_GET_DRVCONF_RDSEL     = 36,
    CAN_GET_DRVCONF_OTSENS    = 38,
    CAN_GET_DRVCONF_SHRTSENS  = 40,
    CAN_GET_DRVCONF_EN_PFD    = 42,
    CAN_GET_DRVCONF_EN_S2VS   = 44,
    CAN_GET_SGCSCONF_SFILT    = 46,
    CAN_GET_SGCSCONF_SGT      = 48,
    CAN_GET_SGCSCONF_CS       = 50,
    CAN_GET_SMARTEN_SEIMIN    = 52,
    CAN_GET_SMARTEN_SEDN      = 54,
    CAN_GET_SMARTEN_SEMAX     = 56,
    CAN_GET_SMARTEN_SEUP      = 58,
    CAN_GET_SMARTEN_SEMIN     = 60,
    CAN_GET_CHOPCONF_TBL      = 62,
    CAN_GET_CHOPCONF_CHM      = 64,
    CAN_GET_CHOPCONF_RNDTF    = 66,
    CAN_GET_CHOPCONF_HDEC     = 68,
    CAN_GET_CHOPCONF_HEND     = 70,
    CAN_GET_CHOPCONF_HSTRT    = 72,
    CAN_GET_CHOPCONF_TOFF     = 74,
    CAN_GET_DRVCTRL_INTPOL    = 76,
    CAN_GET_DRVCTRL_DEDGE     = 78,
    CAN_GET_DRVCTRL_MRES      = 80,

    CAN_SEND_ODOM_ANGLE       = 251,
    CAN_SEND_HEALTH_STATUS    = 252

} RAD_CAN_MSG;


typedef enum __rad_can_id : uint8_t
{
    RAD__DRIVE__FRONT_RIGHT = 0x01,
    RAD__DRIVE__BACK_RIGHT = 0x02,
    RAD__DRIVE__BACK_LEFT = 0x03,
    RAD__DRIVE__FRONT_LEFT = 0x04,

    RAD_TEST_PID__FRONT_RIGHT = 0x14,
    RAD_TEST_PID__FRONT_LEFT = 0x15

} RAD_ID;


typedef enum __rad_gear_ratios : uint16_t
{
    RAD__DRIVE__GEAR_RATIO = 30
    
} RAD_GEAR_RATIO;


class RAD
{
public:
    /**
     * Takes CANraw msg, calling a CAN command updates
     * the CAN frame accordingly with its RAD ID
     * 
     * @param can_msg CANraw message to broadcast
     * @param can_id CAN ID of RAD
     * 
     */
    RAD(CANraw* can_msg);
    RAD(CANraw* can_msg, RAD_ID can_id);

    /**
     * @brief Set angle offset from zero position (in degrees)
     * 
     * @param offset_angle angle offset
     */
    void set_pid_angle_offset(float offset_angle);

    /**
     * @brief Set multiplication factor of angle setting
     * 
     * @param factor multiplication factor
     */
    void set_mul_factor(float factor);


    // CAN COMMANDS
    /**
     * @brief Calibrates zero position of RAD motor
     */
    void calibrate_zero_pos();

    /**
     * @brief Set target angle of RAD motor (in degrees)
     * 
     * @param angle target angle/setpoint
     */
    void set_target_angle(float angle);

    /**
     * @brief Set stepper speed of RAD motor (STEP/DIR)
     * 
     * @param speed speed of motor
     */
    void set_stepper_speed(float speed);

    /**
     * @brief Set kP value of RAD motor
     * 
     * @param P kP value
     */
    void set_p_value(float P);

    /**
     * @brief Set kI value of RAD motor
     * 
     * @param I kI value
     */
    void set_i_value(float I);

    /**
     * @brief Set kD value of RAD motor
     * 
     * @param D kD value
     */
    void set_d_value(float D);

private:
    void _update_can_data(uint8_t* buf, size_t size);

    CANraw* l_can_msg;

    float l_offset, l_factor;
};

float __buffer_get_float32(uint8_t* buf, uint8_t* ind);
void __buffer_append_float32(uint8_t* buf, float n, uint8_t* ind);

/**
 * Decode incoming CAN status frame and update RAD status message
 * 
 * @param can_msg CAN status frame to decode
 * @param status RAD status msg after processing CAN frame
 */
uint8_t decode_can_msg(const CANraw* can_msg, RadStatus* status);
