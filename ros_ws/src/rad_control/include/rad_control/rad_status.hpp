#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/can_raw.hpp"
#include "custom_interfaces/msg/rad_status.hpp"

#include "rad.hpp"

using namespace custom_interfaces::msg;
using std::placeholders::_1;

class RAD_Status : public rclcpp::Node
{
public:
    RAD_Status();

private:
    void _status_callback(const CANraw& msg);
    void _timer_callback(void);

    std::shared_ptr<rclcpp::Publisher<RadStatus>> status_pub_1,
        status_pub_2, status_pub_3, status_pub_4;
    std::shared_ptr<rclcpp::Subscription<CANraw> > can_sub_;
    std::shared_ptr<rclcpp::TimerBase> timer_;

    RadStatus status_1, status_2, status_3, status_4;

    uint8_t rate;
    std::string topic;
};