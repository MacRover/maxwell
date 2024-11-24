#include "arm_controller/rad_converter.hpp"

void RAD_Converter::_callback(const custom_interfaces::msg::RadStatus::SharedPtr msg, int id)
{
  joint_trajectory_.points[0].positions[id] = msg->angle;
  joint_trajectory_.points[0].velocities[id] = msg->speed;
  pub_->publish(joint_trajectory_);    
}

RAD_Converter::RAD_Converter() : Node("rad_arm_converter"),
                                 joints_(NUM_JOINTS, 0.0)
{
  joint_trajectory_.joint_names = { "arm_base_joint", "arm_shoulder_joint", "arm_elbow_joint", "arm_wrist_joint", "gripper_joint"};
  joint_trajectory_.points.resize(1);
  joint_trajectory_.points[0].time_from_start = rclcpp::Duration(1, 0);
  joint_trajectory_.points[0].positions = {0.0, 0.0, 0.0, 0.0, 0.0};
  joint_trajectory_.points[0].velocities = {0.0, 0.0, 0.0, 0.0, 0.0};

  for (int i = 0; i < NUM_JOINTS; i++)
  {
    sub_[i] = this->create_subscription<custom_interfaces::msg::RadStatus>(
        "/joint" + std::to_string(i) + "/rad_status", 10,
        [&](const custom_interfaces::msg::RadStatus::SharedPtr msg){ _callback(msg, i); });
  }
  pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", 1);

}

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RAD_Converter>());
  rclcpp::shutdown();

  return 0;
}