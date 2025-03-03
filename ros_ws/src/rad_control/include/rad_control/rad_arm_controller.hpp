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
  void _callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void publish_joint_angles();   
  void _timer_callback(void);
  void _publish_to_can();

  std::shared_ptr<rclcpp::Publisher<CANraw> > can_pub_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState> > sub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState> > pub_;
  std::shared_ptr<rclcpp::TimerBase> timer_; 

  CANraw can_base, can_shoulder, can_gripper, can_elbow, can_wrist, can_pitch;


  RAD rad_base_arm, rad_pitch_arm, rad_gripper_arm, rad_shoulder_arm, rad_elbow_arm, rad_wrist_arm; 

  uint16_t sleep_msec;

};
