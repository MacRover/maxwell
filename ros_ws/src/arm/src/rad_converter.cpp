#include "arm_controller/rad_converter.hpp"
#include "arm_controller/arm_constants.hpp"
#include <cmath>
#include <iostream>


RAD_Converter::RAD_Converter() : Node("rad_arm_converter"),
                                 joints_(NUM_JOINTS, 0.0)
{
  joint_state_.name = {"arm_base_joint", "arm_shoulder_joint", "arm_elbow_joint", "arm_wrist_joint", "gripper_joint"};
  joint_state_.position.assign(5, 0);
  ;
  joint_state_.velocity.assign(5, 0);
  ;

  for (int i = 0; i < NUM_JOINTS; i++)
  {
    sub_[i] = this->create_subscription<custom_interfaces::msg::RadStatus>(
        "/arm/joint" + std::to_string(i) + "/rad_status", 10,
        [this, i](const custom_interfaces::msg::RadStatus::SharedPtr msg)
        {
          switch (i)
          {
          case 1: // Shoulder
            joint_state_.position[i] = acos((a_shoulder*a_shoulder + b_shoulder*b_shoulder - pow((0.00254 / 360 * (13320 - msg->angle) + lmin_shoulder), 2)) / (2 * a_shoulder * b_shoulder));
            joint_state_.position[i] -= shoulder_offset;
            std::cout << "Shoulder: " << joint_state_.position[i]*180/M_PI << std::endl;
            break;
          case 2: // Elbow
            joint_state_.position[i] = acos((a_elbow*a_elbow + b_elbow*b_elbow - pow((0.00254 / 360 * (8297 - msg->angle) + lmin_elbow), 2)) / (2 * a_elbow * b_elbow));
            joint_state_.position[i] -= elbow_offset;
            std::cout << "Elbow: " << joint_state_.position[i]*180/M_PI << std::endl;
            break;
          default:
            joint_state_.position[i] = 0.0;
            break;
          }
          // joint_state_.velocity[i] = msg->speed;
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