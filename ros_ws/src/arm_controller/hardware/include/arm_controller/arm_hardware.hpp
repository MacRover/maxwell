
#ifndef ARM_CONTROLLER__ARM_HARDWARE_HPP_
#define ARM_CONTROLLER__ARM_HARDWARE_HPP_

#include "string"
#include "unordered_map"
#include "vector"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::return_type;
using sensor_msgs::msg::JointState;
using std_msgs::msg::Float64;

namespace arm_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HARDWARE_INTERFACE_PUBLIC RobotSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:
  /// The size of this vector is (standard_interfaces_.size() x nr_joints)
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocities_command_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocities_;
  sensor_msgs::msg::JointState last_joint_state_;
  Float64 gripper_angle;

  std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
    {"position", {}}, {"velocity", {}}};
  };

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<Float64>::SharedPtr gripper_sub_;
  JointState joint_state_msg_;


}  // namespace arm_controller

#endif  // ARM_CONTROLLER__ARM_HARDWARE_HPP_