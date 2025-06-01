#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <stdio.h>
#include <memory.h>
#include <unistd.h>
#include <vector>



class ArmRadDummy
{
public:
  ArmRadDummy(std::shared_ptr<rclcpp::Node> nh);
  void keyLoop();

private:
  
  std::shared_ptr<rclcpp::Node> nh_;
  std::vector<double> joints_;
  sensor_msgs::msg::JointState joint_state_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
};

ArmRadDummy::ArmRadDummy(std::shared_ptr<rclcpp::Node> nh):
  nh_(nh),
  joints_(6, 0.0)
{
  joint_state_.name.resize(5);
  joint_state_.name = { "arm_base_joint", "arm_shoulder_joint", "arm_elbow_joint", "arm_wrist_joint", "gripper_joint"};
  joint_state_.position.resize(6);
  joint_pub_ = nh_->create_publisher<sensor_msgs::msg::JointState>("/arm/rad/joint_states", 10);
  joint_sub_ = nh_->create_subscription<sensor_msgs::msg::JointState>("/arm/hardware/joint_states", 10, [&](sensor_msgs::msg::JointState::SharedPtr msg) {
    joint_state_ = *msg;
  });


  timer_ = nh_->create_wall_timer(std::chrono::milliseconds(1000 / 10), [this]() -> void {
    joint_pub_->publish(joint_state_);
  });
}


int main(int argc, char** argv)
{
//   ros::init(argc, argv, "teleop_turtle");
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("arm_rad_dummy");
  ArmRadDummy rad_dummy(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return(0);
}