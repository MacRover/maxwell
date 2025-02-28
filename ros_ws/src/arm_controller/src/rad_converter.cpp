#include "arm_controller/rad_converter.hpp"

RAD_Converter::RAD_Converter() : Node("rad_arm_converter"),
                                 joints_(NUM_JOINTS, 0.0)
{
  joint_state_.name = { "arm_base_joint", "arm_shoulder_joint", "arm_elbow_joint", "arm_wrist_joint", "gripper_joint"};
  joint_state_.position.assign(5, 0);;
  joint_state_.velocity.assign(5, 0);;

  for (int i = 0; i < NUM_JOINTS; i++)
  {
    sub_[i] = this->create_subscription<custom_interfaces::msg::RadStatus>(
        "/joint" + std::to_string(i) + "/rad_status", 10,
        [this, i](const custom_interfaces::msg::RadStatus::SharedPtr msg){
          joint_state_.position[i] = msg->angle;
          joint_state_.velocity[i] = msg->speed;
          pub_->publish(joint_state_);    
        });
  }
  pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/arm/rad/joint_states", 1);

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