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
    void _status_callback(const CANraw& msg);
    void _timer_callback(void);

    std::shared_ptr<rclcpp::Publisher<RadStatus>> status_pub;
    std::shared_ptr<rclcpp::Subscription<CANraw> > can_sub_;
    std::shared_ptr<rclcpp::TimerBase> timer_;
    RadStatus status;

    uint8_t can_id, rate;
    std::string ns, topic;
};