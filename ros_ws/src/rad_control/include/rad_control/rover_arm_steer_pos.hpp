#pragma once
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include "custom_interfaces/msg/ca_nraw.hpp"

#include "rad.hpp"

using namespace custom_interfaces::msg;
using namespace trajectory_msgs::msg;
using std::placeholders::_1;
using namespace std::chrono_literals;

class ArmSteerPos : public rclcpp::Node
{
public:
    ArmSteerPos();
private:
    void _callback(const JointTrajectory& msg);
    void _timer_callback(void);
    std::shared_ptr<rclcpp::Publisher<CANraw> > can_pub_;
    std::shared_ptr<rclcpp::Subscription<JointTrajectory> > sub_;
    std::shared_ptr<rclcpp::TimerBase> timer_;

    rclcpp::Rate rate;

    RAD rad_base, rad_shoulder, rad_elbow;

    CANraw can_msg_base, can_msg_elbow, can_msg_shoulder;
};