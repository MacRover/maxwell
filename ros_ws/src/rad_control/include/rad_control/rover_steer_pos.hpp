#pragma once
#include <vector>
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "custom_interfaces/msg/ca_nraw.hpp"
#include "custom_interfaces/msg/swerve_module_pulse.hpp"

#include "rad.hpp"

using namespace custom_interfaces::msg;
using namespace std_msgs::msg;
using std::placeholders::_1;
using namespace std::chrono_literals;

class SteerPos : public rclcpp::Node
{
public:
    SteerPos(std::string name);
private:
    void _timer_callback(void);
    void _pulse_callback(const SwerveModulePulse& msg);
    std::shared_ptr<rclcpp::Publisher<CANraw> > can_pub_;
    std::shared_ptr<rclcpp::Subscription<SwerveModulePulse> > sub_;
    std::shared_ptr<rclcpp::TimerBase> timer_;
    CANraw can_msg_1, can_msg_2, can_msg_3, can_msg_4;

    RAD rad_fr, rad_fl, rad_br, rad_bl;

    uint16_t sleep_msec;
};