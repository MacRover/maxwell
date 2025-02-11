#pragma once
#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/ca_nraw.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rad.hpp"


class RAD_Arm_Controller : public rclcpp::Node
{
public:
  RAD_Arm_Controller();
private:
  void _callback(const JointState& msg);
  void publish_joint_angles(const JointState& msg);   
  void _timer_callback(void);

  std::shared_ptr<rclcpp::Subscription<JointState> > sub_;
  std::shared_ptr<rclcpp::Publisher<JointState> > pub_;
  std::shared_ptr<rclcpp::TimerBase> timer_; 

  RAD rad_base_arm, rad_elbow_arm, rad_shoulder_arm, rad_pitch_arm, rad_wrist_arm, rad_gripper_arm; 
};
