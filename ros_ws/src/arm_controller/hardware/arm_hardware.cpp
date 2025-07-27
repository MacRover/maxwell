// Copyright 2023 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "arm_controller/arm_hardware.hpp"

#include <iostream>

namespace arm_controller
{


  
CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // robot has 5 joints and 2 interfaces
  joint_position_.assign(5, 0);
  joint_velocities_.assign(5, 0);
  joint_position_command_.assign(5, 0);
  joint_velocities_command_.assign(5, 0);


  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : joint.state_interfaces)
    {
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }

  for (auto i = 0ul; i < joint_position_command_.size(); i++)
  {
    // Change code below in future to be more flexible
    joint_position_[i] = joint_position_command_[i] = std::stod(info_.joints[i].state_interfaces[0].initial_value);
  }

  node_ = std::make_shared<rclcpp::Node>("arm_hardware_interface");
  joint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/arm/hardware/joint_states", 10);

  joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/arm/rad/joint_states", 10,
    [&](sensor_msgs::msg::JointState::SharedPtr msg) {
      last_joint_state_ = *msg;
    });
  
  gripper_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    "/arm/gripper/angle", 10,
    [&](std_msgs::msg::Float64::SharedPtr msg) {
      gripper_angle = *msg;
    });

  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
  }

  return command_interfaces;
}

return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{

  rclcpp::spin_some(node_);
  // TODO velocity

  // for (auto i = 0ul; i < joint_velocities_command_.size(); i++)
  // {
  //   joint_velocities_[i] = joint_velocities_command_[i];
  //   joint_position_[i] += joint_velocities_command_[i] * period.seconds();
  // }

  for (auto i = 0ul; i < joint_position_command_.size(); i++)
  {
    // joint_position_[i] = last_joint_state_.position[i];
    joint_position_[i] = joint_position_command_[i];
  }

  return return_type::OK;
}

return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  joint_state_msg_.header.stamp = node_->now();
  joint_state_msg_.name = joint_interfaces["position"];
  joint_state_msg_.name.push_back("finger_joint");
  joint_state_msg_.position = joint_position_command_;
  joint_state_msg_.position.push_back(gripper_angle.data);

  joint_pub_->publish(joint_state_msg_);

  return return_type::OK;
}

}  // namespace arm_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  arm_controller::RobotSystem, hardware_interface::SystemInterface)
