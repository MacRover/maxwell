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
    CAN_PULSE_STEPPER         = 81,

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
    CAN_ASSIGN_DEVICE_ID      = 85,
    CAN_SET_PID_MIN_OUTPUT = 87,
    CAN_GET_PID_MIN_OUTPUT = 88,
    CAN_SET_PID_MAX_OUTPUT      = 89,
    CAN_GET_PID_MAX_OUTPUT      = 90,
    CAN_SET_HOME_OFFSET         = 91,
    CAN_GET_HOME_OFFSET         = 92,
    CAN_SET_MAX_POINT           = 93,
    CAN_GET_MAX_POINT           = 94,
    CAN_SET_ZERO_POINT          = 95,
    CAN_GET_ZERO_POINT          = 96,
    CAN_SET_RAD_FLAGS           = 97,
    CAN_GET_RAD_FLAGS           = 98,


    CAN_SEND_ODOM_ANGLE       = 251,
    CAN_SEND_HEALTH_STATUS    = 252

} RAD_CAN_MSG;

typedef enum __global_can_msg : uint8_t
{
    CAN_ESTOP_MESSAGE = 0x31,
    CAN_DISABLE_MESSAGE = 0x00,
    CAN_ENABLE_MESSAGE = 0x02,
    CAN_HEALTH_STATUS_PING = 0x03
} GLOBAL_CAN_MSG;


typedef enum __rad_can_id : uint8_t
{
    RAD__DRIVE__FRONT_RIGHT = 0x11,
    RAD__DRIVE__BACK_RIGHT = 0x14,
    RAD__DRIVE__BACK_LEFT = 0x13,
    RAD__DRIVE__FRONT_LEFT = 0x12,

    RAD__ARM__BASE     = 0x15,
    RAD__ARM__SHOULDER = 0x16,
    RAD__ARM__ELBOW    = 0x17,
    RAD__ARM__WRIST_LS = 0x20,
    RAD__ARM__WRIST_RS = 0x18,
    RAD__ARM__GRIPPER = 0x19

} RAD_ID;


typedef enum __rad_gear_ratios : uint16_t
{
    RAD__DRIVE__GEAR_RATIO = 30
    
} RAD_GEAR_RATIO;

typedef enum __rad_status : uint8_t
{
    OK = 0x00U,
    ERROR = 0x01U,
    BUSY = 0x02U,
    TIMEOUT = 0x03U
} RAD_STATUS;


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
    RAD(CANraw* can_msg, uint8_t can_id);


    /**
     * @brief Set CAN ID of RAD
     * 
     * @param can_id new RAD CAN ID
     */
    void set_can_id(uint8_t can_id);

    /**
     * @brief Set angle offset from zero position (in degrees)
     * 
     * @param offset_angle angle offset
     */
    void set_pid_angle_offset(double offset_angle);

    /**
     * @brief Set multiplication factor of angle setting
     * 
     * @param factor multiplication factor
     */
    void set_mul_factor(double factor);


    // CAN COMMANDS
    /**
     * @brief Calibrates zero position of RAD motor
     */
    void calibrate_zero_pos();

    /**
     * @brief Cancel calibration of RAD motor
     * 
     */
    void cancel_calibration();

    /**
     * @brief Set target angle of RAD motor (in degrees)
     * 
     * @param angle target angle/setpoint
     */
    void set_target_angle(double angle);

    /**
     * @brief Set stepper speed of RAD motor (STEP/DIR)
     * 
     * @param speed speed of motor
     */
    void set_stepper_speed(float speed);

    /**
     * @brief Get stepper speed of RAD motor (STEP/DIR)
     */
    void get_stepper_speed();

    /**
     * @brief Set kP value of RAD motor
     * 
     * @param P kP value
     */
    void set_p_value(double P);

    /**
     * @brief Set kI value of RAD motor
     * 
     * @param I kI value
     */
    void set_i_value(double I);

    /**
     * @brief Set kD value of RAD motor
     * 
     * @param D kD value
     */
    void set_d_value(double D);

    /**
     * @brief Set RAD type of RAD
     *
     * @param type RAD type
     */
    void set_rad_type(uint8_t type);

    /**
     * @brief Get RAD type of RAD
     */
    void get_rad_type();

    /**
     * @brief Get target angle of RAD motor (in deg)
     *   
     */
    void get_target_angle();

    /**
     * @brief Get kP value of RAD motor
     */
    void get_p_value();

    /**
     * @brief Get kI value of RAD motor
     */
    void get_i_value();

    /**
     * @brief Get kD value of RAD motor
     */
    void get_d_value();

    /**
     * @brief Set odometry/angle message interval
     * 
     * @param period period of messages (in ms)
     */
    void set_odom_interval(uint32_t period);

    /**
     * @brief Get odometry/angle message interval
     */
    void get_odom_interval();

    /**
     * @brief Set status message interval
     * 
     * @param period period of messages (in ms)
     */
    void set_health_interval(uint32_t period);

    /**
     * @brief Get status message interval
     */
    void get_health_interval();

    /**
     * @brief Set TEST mode to expose digital test values
     * 
     * @param enabled 0 for disable, 1 for enable
     */
    void set_drvconf_tst(bool enabled);

    /**
     * @brief Get TEST mode to expose digital test values
     */
    void get_drvconf_tst();

    /**
     * @brief Set slope control
     * 
     * @param slope 0 for minimum slope driver strength, 7 for maximum
     */
    void set_drvconf_slp(uint8_t slope);

    /**
     * @brief Get slope control
     */
    void get_drvconf_slp();

    /**
     * @brief Set short to GND protection state
     * 
     * @param disabled 0 for enabled, 1 for disabled
     */
    void set_drvconf_s2g(bool disabled);

    /**
     * @brief Get short to GND protection state
     */
    void get_drvconf_s2g();

    /**
     * @brief Set short detection delay for high-side and low-side FETs
     * 
     * @param delay b00 - 3.2 microsec
     *              b01 - 1.6 microsec
     *              b10 - 1.2 microsec
     *              b11 - 0.8 microsec
     */
    void set_drvconf_ts2g(uint8_t delay);

    /**
     * @brief Get short detection delay
     */
    void get_drvconf_ts2g();

    /**
     * @brief Enable/disable STEP DIR
     * 
     * @param disabled 0 for enabled, 1 for disabled
     */
    void set_drvconf_sdoff(bool disabled);

    /**
     * @brief Get STEP DIR interface state
     */
    void get_drvconf_sdoff();

    /**
     * @brief Adjust sense resistor voltage between low and high sensitivity (high and low resistance respectively)
     * 
     * @param enabled 0 for higher sense voltage (.325V), 1 for lower (.173V)
     */
    void set_drvconf_vsense(bool enabled);

    /**
     * @brief Get sense resistor voltage bit
     */
    void get_drvconf_vsense();

    /**
     * @brief Set read out from TMC driver
     * 
     * @param readout value for readout type
     *                b00 - Microstep position read back
     *                b01 - StallGuard2 level read back
     *                b10 - StallGuard2 and CoolStep current level read back
     *                b11 - status flags and detectors
     */
    void set_drvconf_rdsel(uint8_t readout);

    /**
     * @brief Get read out type from TMC driver
     */
    void get_drvconf_rdsel();

    /**
     * @brief Set overtemperature sensitivity
     * 
     * @param lower_shutdown 0 for shutdown at 150C, 1 for shutdown at 136C
     */
    void set_drvconf_otsens(bool lower_shutdown);

    /**
     * @brief Get overtemperature sensitivity state
     */
    void get_drvconf_otsens();
    
    /**
     * @brief Set short to GND detection sensitivity
     * 
     * @param sensitive 0 for not sensitive, 1 for sensitive
     */
    void set_drvconf_shrtsens(bool sensitive);

    /**
     * @brief Get short to GND detection sensitivity
     */
    void get_drvconf_shrtsens();

    /**
     * @brief Enable/disable passive fast decay (for reducing motor vibrations)
     * 
     * @param enabled 0 for no additional motor dampening, 1 for motor dampening
     */
    void set_drvconf_en_pfd(bool enabled);

    /**
     * @brief Get passive fast decay mode
     */
    void get_drvconf_en_pfd();

    /**
     * @brief Enable/disable short to VS protection
     * 
     * @param enabled 0 for no protection, 1 for protection
     */
    void set_drvconf_en_s2vs(bool enabled);

    /**
     * @brief Get short to VS protection mode
     */
    void get_drvconf_en_s2vs();

    /**
     * @brief Set minimum CoolStep current
     * 
     * @param lower_current 0 - 1/2 CS current setting, 1 - 1/4 CS
     */
    void set_smarten_seimin(bool lower_current);

    /**
     * @brief Get minimum CoolStep current
     */
    void get_smarten_seimin();

    /**
     * @brief Set current decrement speed (num of occurrences of SG measurements above threshold until current decreases)
     * 
     * @param samples     SG number of samples
     *                    b00 - 32
     *                    b01 - 8
     *                    b10 - 2
     *                    b11 - 1
     */
    void set_smarten_sedn(uint8_t samples);

    /**
     * @brief Get current decrement speed
     */
    void get_smarten_sedn();

    /**
     * @brief Set current increment speed (num of occurrences of SG measurements below threshold until current increases)
     * 
     * @param samples     SG number of samples
     *                    b00 - 32
     *                    b01 - 8
     *                    b10 - 2
     *                    b11 - 1
     */
    void set_smarten_seup(uint8_t samples);

    /**
     * @brief Get current increment speed
     */
    void get_smarten_seup();

    /**
     * @brief Set lower threshold setting for SG. If current is below threshold, increase current
     * 
     * @param threshold 4 bit unsigned int for min threshold
     */
    void set_smarten_semin(uint8_t threshold);

    /**
     * @brief Get lower threshold setting for SG
     */
    void get_smarten_semin();

    /**
     * @brief Set upper threshold setting for SG. If current is above threshold, decrease current
     * 
     * @param threshold 4 bit unsigned int for max threshold
     */
    void set_smarten_semax(uint8_t threshold);

    /**
     * @brief Get upper threshold setting for SG
     */
    void get_smarten_semax();

    /**
     * @brief Set blanking time
     * 
     * @param interval Blanking time interval, in system clock periods
     *                 b00 - 16
     *                 b01 - 24
     *                 b10 - 36
     *                 b11 - 54
     */
    void set_chopconf_tbl(uint8_t interval);

    /**
     * @brief Get blanking time setting
     */
    void get_chopconf_tbl();
    
    /**
     * @brief Set chopper mode
     * 
     * @param mode 0 for standard mode, 1 for constant TOFF with fast decay time
     */
    void set_chopconf_chm(bool mode);

    /**
     * @brief Get chopper mode
     */
    void get_chopconf_chm();

    /**
     * @brief Enable/disable random TOFF
     * 
     * @param enabled 0 for fixed TOFF time, 1 for random mode
     */
    void set_chopconf_rndtf(bool enabled);

    /**
     * @brief Get random TOFF mode
     */
    void get_chopconf_rndtf();

    /**
     * @brief Set hysteresis decrement interval or fast decay mode
     * 
     * @param value 2-bit value. 
     *              When chopper mode enabled, MSB indicating fast decay time
     *              When chopper mode disabled, Hysteresis decrement period setting, in system clock periods
     *              b00 - 16
     *              b01 - 32
     *              b10 - 48
     *              b11 - 64
     */
    void set_chopconf_hdec(uint8_t value);

    /**
     * @brief Get hysteresis decrement interval or fast decay mode
     */
    void get_chopconf_hdec();

    /**
     * @brief Set hysteresis end value or sine wave offset
     * 
     * @param value 4-bit value.
     */
    void set_chopconf_hend(uint8_t value);

    /**
     * @brief Get hysteresis end value or sine wave offset
     */
    void get_chopconf_hend();

    /**
     * @brief Set hysteresis start value or fast decay time setting
     * 
     * @param value 3-bit value.
     *              When chopper mode enabled, 3 LSBs specify duration of fast decay phase
     *              When chopper mode disabled, hysteresis start offset from HEND (1 to 8)
     */
    void set_chopconf_hstrt(uint8_t value);

    /**
     * @brief Get hysteresis start value or fast decay time setting
     * 
     */
    void get_chopconf_hstrt();

    /**
     * @brief Set the TOFF time
     * 
     * @param toff toff from 1-15; if toff is 0, MOSFETs are disabled
     */
    void set_chopconf_toff(uint8_t toff);

    /**
     * @brief Get the TOFF time
     */
    void get_chopconf_toff();

    /**
     * @brief Enable/Disable MicroPlyer Step interpolator
     * 
     * @param enabled 0 for disable, 1 for enable
     */
    void set_drvctrl_intpol(bool enabled);

    /**
     * @brief Get MicroPlyer Step interpolator state
     */
    void get_drvctrl_intpol();

    /**
     * @brief Enable/Disable double edge STEP pulses
     * 
     * @param enabled 0 for disable, 1 for enable
     */
    void set_drvctrl_dedge(bool enabled);

    /**
     * @brief Get DEDGE state
     */
    void get_drvctrl_dedge();

    /**
     * @brief Set Microstep Resolution of RAD motor
     *
     * @param mres microstep resolution
     */
    void set_drvctrl_mres(uint8_t mres);

    /**
     * @brief Get Microstep Resolution of RAD motor
     */
    void get_drvctrl_mres();

    /**
     * @brief Enable/disable StallGuard2 filter
     * 
     * @param filtered 0 for standard mode, 1 for filtered mode
     */
    void set_sgcsconf_sfilt(bool filtered);

    /**
     * @brief Get StallGuard2 filter mode
     */
    void get_sgcsconf_sfilt();

    /**
     * @brief Set StallGuard2 threshold value for controlling stall indicator output
     * 
     * @param threshold 6-bit signed integer for stallguard threshold
     */
    void set_sgcsconf_sgt(int8_t threshold);

    /**
     * @brief Get StallGuard2 threshold value for controlling stall indicator output
     */
    void get_sgcsconf_sgt();

    /**
     * @brief Set Current Scale of RAD motor
     * 
     * @param cs current scale
     */
    void set_sgcsconf_cs(uint8_t cs);

    /**
     * @brief Get Current Scale of RAD motor
     */
    void get_sgcsconf_cs();

    /**
     * @brief Save to EEPROM
     * 
     */
    void save_to_eeprom();

    /**
     * @brief Reload from EEPROM
     */
    void reload_from_eeprom();

    /**
     * @brief Set PID Min Output
     * 
     * @param min_output pid min output
     */
    void set_min_output(uint16_t min_output);

    /**
     * @brief Get PID Min Output
     * 
     */
    void get_min_output();

    /**
     * @brief Set PID Max Output
     * 
     * @param max_output pid max output
     */
    void set_max_output(uint16_t max_output);

    /**
     * @brief Get PID Max Output
     */
    void get_max_output();

    /**
     * @brief Pulse motor with given number of steps
     * 
     * @param steps number of steps to move
     */
    void pulse_stepper(float steps);

    /**
     * @brief Set home position of RAD
     * 
     * @param pos home position
     */
    void set_home_position(uint32_t pos);

    /**
     * @brief Get the home position of RAD
     */
    void get_home_position();

    /**
     * @brief Set encoder offset for RAD motor
     * 
     */
    void set_home_offset();

    /**
     * @brief Get encoder offset for RAD motor
     * 
     */
    void get_home_offset();

    /**
     * @brief Set zero point for RAD encoder
     * 
     */
    void set_zero_point();

    /**
     * @brief Set max point for RAD encoder
     * 
     * @param rollover_count rollover count (max point = rollover_count * MOTOR_GEARING)
     */
    void set_max_point(uint8_t rollover_count);

    /**
     * @brief Set RAD config flags
     *
     */
    void set_rad_flags(uint8_t flags);

    /**
     * @brief Get RAD config flags
     * 
     */
    void get_rad_flags();

private:
    void _set_null_data(RAD_CAN_MSG msg);
    void _update_can_data(uint8_t* buf, size_t size);

    CANraw* l_can_msg;

    uint8_t l_can_id;

    double l_offset, l_factor;
};

float __buffer_get_float32(uint8_t* buf, uint8_t* ind);
double __buffer_get_float64(uint8_t* buf, uint8_t* ind);
uint32_t __buffer_get_uint32(uint8_t* buf, uint8_t* ind);
uint64_t __buffer_get_uint64(uint8_t* buf, uint8_t* ind);

void __buffer_append_float32(uint8_t* buf, float n, uint8_t* ind);
void __buffer_append_float64(uint8_t* buf, double n, uint8_t* ind);
void __buffer_append_uint32(uint8_t* buf, uint32_t n, uint8_t* ind);
void __buffer_append_uint64(uint8_t* buf, uint64_t n, uint8_t* ind);

/**
 * Decode incoming CAN status frame and update RAD status message
 * 
 * @param can_msg CAN status frame to decode
 * @param status RAD status msg after processing CAN frame
 */
uint8_t decode_can_msg(const CANraw* can_msg, RadStatus* status);
