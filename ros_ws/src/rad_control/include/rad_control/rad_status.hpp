#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/ca_nraw.hpp"
#include "custom_interfaces/msg/rad_status.hpp"

#include "rad.hpp"

using namespace custom_interfaces::msg;
using std::placeholders::_1;

class RAD_Status : public rclcpp::Node
{
public:
    RAD_Status();

private:
    void _status_callback_1(const CANraw& msg);
    void _status_callback_2(const CANraw& msg);
    void _status_callback_3(const CANraw& msg);
    void _timer_callback(void);

    void process_can_msg(const CANraw& msg, 
                        uint8_t (*decode_can_msg)(const CANraw* can_msg, RadStatus* status));

    std::shared_ptr<rclcpp::Publisher<RadStatus>> status_pub_1,
        status_pub_2, status_pub_3, status_pub_4;
    std::shared_ptr<rclcpp::Subscription<CANraw> > can_sub_1, can_sub_2, can_sub_3;
    std::shared_ptr<rclcpp::TimerBase> timer_;

    RadStatus status_1, status_2, status_3, status_4;

    uint8_t rate;
    std::string topic_1, topic_2, topic_3;
};