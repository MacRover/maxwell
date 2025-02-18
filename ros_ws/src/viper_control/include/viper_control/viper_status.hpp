#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/ca_nraw.hpp"
#include "custom_interfaces/msg/viper_status.hpp"
#include "custom_interfaces/msg/viper_card_status.hpp"

#include "viper.hpp"

using namespace custom_interfaces::msg;
using std::placeholders::_1;

class VIPER_Status : public rclcpp::Node
{
public:
    VIPER_Status();

private:
    void _status_callback(const CANraw& msg);
    void _timer_callback(void);

    std::shared_ptr<rclcpp::Publisher<ViperCardStatus>> status_pub_card_0,
        status_pub_card_1, status_pub_card_2, status_pub_card_3;

    std::shared_ptr<rclcpp::Publisher<ViperStatus>> status_pub;

    std::shared_ptr<rclcpp::Subscription<CANraw> > can_sub_;
    std::shared_ptr<rclcpp::TimerBase> timer_;

    ViperStatus status;
    ViperCardStatus card_0_status, card_1_status, card_2_status, card_3_status;

    uint8_t rate;
    std::string topic;
};