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

class RAD_Status_Configure : public rclcpp::Node
{
public:
    RAD_Status_Configure();

private:
   
    std::shared_ptr<rclcpp::Pulisher<CANraw> > can_pub_;

    std::string topic;

    CANraw pub;
};