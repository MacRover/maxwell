#pragma once
#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/ca_nraw.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rad.hpp"

using namespace custom_interfaces::msg;

class RAD_Arm_Controller : public rclcpp::Node
{
public:
  RAD_Arm_Controller();
private:
  void _callback(const sensor_msgs::msg::JointState& msg);
  void _publish_to_can();
  std::shared_ptr<rclcpp::Publisher<CANraw> > can_pub_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState> > sub_;

  CANraw can_base, can_shoulder, can_gripper, can_elbow, can_rs, can_ls;

  RAD rad_base_arm, rad_ls_arm, rad_gripper_arm, rad_shoulder_arm, rad_elbow_arm, rad_rs_arm; 

  uint16_t sleep_msec;

  std::vector<double> lmins, a_lengths, b_lengths, offsets; 

  double screw_max;
};
