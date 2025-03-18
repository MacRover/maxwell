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

    std::vector<std::shared_ptr<rclcpp::Publisher<RadStatus>>> status_pub;
    std::vector<RadStatus> status_topic;
    std::shared_ptr<rclcpp::Subscription<CANraw> > can_sub_;
    std::shared_ptr<rclcpp::TimerBase> timer_;

    uint8_t rate;
    std::string topic;

    std::vector<int64_t> rad_ids;
    std::vector<std::string> rad_status;
};