#pragma once
#include <vector>
#include <byteswap.h>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/ca_nraw.hpp"

#define BIG_ENDIANNESS
#define CAN_MESSAGE_IDENTIFIER_VIPER 0x02
#define CAN_MESSAGE_IDENTIFIER_OFFSET 25


using namespace custom_interfaces::msg;

typedef enum VIPER_CAN_MSG : uint8_t
{
    CAN_SET_HEALTH_INTERVAL      = 11,
    CAN_SET_CARD_INTERVAL        = 13,
// ------------------------------------
    CAN_GET_CARD_INTERVAL        = 10,
    CAN_GET_CARD_INTERVAL        = 12,
// ------------------------------------
    SEND_CARD_INPUT_FAULT        = 242,
    SEND_CARD_OUTPUT_A_FAULT     = 243,
    SEND_CARD_OUTPUT_B_FAULT     = 244,
    SEND_CARD_TEMPERATURE        = 245,
    SEND_CARD_INPUT_CURRENT      = 246, 
    SEND_CARD_OUTPUT_DIAGNOSTIC_A = 247, 
    SEND_CARD_OUTPUT_POWER_A     = 248, 
    SEND_CARD_OUTPUT_CURRENT_A   = 249, 
    SEND_CARD_OUTPUT_VOLTAGE_A   = 250, 
    SEND_CARD_OUTPUT_DIAGNOSTIC_B = 251, 
    SEND_CARD_OUTPUT_POWER_B      = 252, 
    SEND_CARD_OUTPUT_CURRENT_B    = 253, 
    SEND_CARD_OUTPUT_VOLTAGE_B    = 254, 
    SEND_HEALTH_STATUS            = 255 
} VIPER_CAN_MSG;

typedef enum __global_can_msg : uint8_t
{
    CAN_ESTOP_MESSAGE = 0x31,
    CAN_DISABLE_MESSAGE = 0x00,
    CAN_ENABLE_MESSAGE = 0x02,
    CAN_HEALTH_STATUS_PING = 0x03
} GLOBAL_CAN_MSG;


typedef enum __VIPER_can_id : uint8_t
{
    VIPER__CARD__0 = 0b100,
    VIPER__CARD__1 = 0b101,
    VIPER__CARD__2 = 0b110,
    VIPER__CARD__3 = 0b111,

} VIPER_ID;


typedef enum __VIPER_status : uint8_t
{
    OK = 0x00U,
    ERROR = 0x01U,
    BUSY = 0x02U,
    TIMEOUT = 0x03U
} VIPER_STATUS;


class VIPER
{
public:
    /**
     * Takes CANraw msg, calling a CAN command updates
     * the CAN frame accordingly with its VIPER ID
     * 
     * @param can_msg CANraw message to broadcast
     * @param can_id CAN ID of VIPER
     * 
     */
    VIPER(CANraw* can_msg);
    VIPER(CANraw* can_msg, uint8_t can_id);

    /**
     * @brief Set CAN ID of VIPER
     * 
     * @param can_id new VIPER CAN ID
     */
    void set_can_id(uint8_t can_id);

    /**
     * @brief Set health interval
     * 
     * @param health_interval new health interval
     */
    void set_health_interval(uint16_t health_interval);

    /**
     * @brief Set card interval
     * 
     * @param card_interval new card interval
     */
    void set_card_interval(uint16_t card_interval);

    /**
     * @brief Sets mux value
     * 
     * @param mux_value new card interval
     */
    void set_mux_value(uint8_t mux_value);

    /**
     * @brief Disable a specific card
     * 
     * @param card VIPER Card ID
     */
    void disable_card(uint8_t card);

    void disable_all_cards();

    /**
     * @brief Enable a specific card
     * 
     * @param card_id VIPER Card ID
     */
    void enable_card(uint8_t card_id);

    void enable_all_cards();

    void get_all_card_data();

    /**
     * @brief Access a specific card's data
     * 
     * @param target VIPER Card input
     */
    void get_card_data(uint8_t target);

    void save_to_eeprom();

private:
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
 * Decode incoming CAN status frame and update VIPER status message
 * 
 * @param can_msg CAN status frame to decode
 * @param status VIPER status msg after processing CAN frame
 */
uint8_t decode_can_msg(const CANraw* can_msg, VIPERStatus* status);