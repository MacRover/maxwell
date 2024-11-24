#pragma once
#include <vector>
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "custom_interfaces/msg/ca_nraw.hpp"

using namespace custom_interfaces::msg;
using namespace std_msgs::msg;
using std::placeholders::_1;
using namespace std::chrono_literals;

class SteerTest : public rclcpp::Node
{
public:
    SteerTest(std::string name);
private:
    void _callback_fr(const Int64& msg);
    void _callback_fl(const Int64& msg);
    void _timer_callback(void);
    std::shared_ptr<rclcpp::Publisher<CANraw> > can_pub_;
    std::shared_ptr<rclcpp::Subscription<Int64> > sub_fr_, sub_fl_;
    std::shared_ptr<rclcpp::TimerBase> timer_;
    CANraw can_msg_1, can_msg_2;
};