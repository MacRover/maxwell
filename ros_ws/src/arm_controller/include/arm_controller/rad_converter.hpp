#include <cstdint>
#include <vector>
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/rad_status.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include "sensor_msgs/msg/joint_state.hpp"

#define NUM_JOINTS 5

using std::placeholders::_1;

class RAD_Converter : public rclcpp::Node
{
public:
  RAD_Converter();

private:
  std::shared_ptr<rclcpp::Subscription<custom_interfaces::msg::RadStatus>> sub_[NUM_JOINTS];
  std::shared_ptr<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>> pub_;
  std::vector<double> joints_;
  trajectory_msgs::msg::JointTrajectory joint_trajectory_;
  sensor_msgs::msg::JointState joint_state_;

  void _callback(const custom_interfaces::msg::RadStatus::SharedPtr msg, int id);

};
