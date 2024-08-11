#pragma once
#include <vector>
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "custom_interfaces/msg/ca_nraw.hpp"
#include "rad.hpp"

using namespace custom_interfaces::msg;
using namespace std_msgs::msg;
using std::placeholders::_1;
using namespace std::chrono_literals;

class SteerTestPID : public rclcpp::Node
{
public:
    SteerTestPID();
private:
    void _callback(const Float32& msg);
    void _timer_callback(void);
    std::shared_ptr<rclcpp::Publisher<CANraw> > can_pub_;
    std::shared_ptr<rclcpp::Subscription<Float32> > sub_;
    std::shared_ptr<rclcpp::TimerBase> timer_;
    CANraw can_fr, can_fl;

    RAD rad_fl, rad_fr;
};