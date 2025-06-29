#include <cstdio>
#include <chrono>
#include "rad_control/rad_drive_controller.hpp"

using std::placeholders::_1;

RAD_Drive_Controller::RAD_Drive_Controller() : 
  Node("rad_drive_controller"),
  rad_fl_drive(&can1_raw, RAD__DRIVE__FRONT_LEFT),
  rad_fr_drive(&can2_raw, RAD__DRIVE__FRONT_RIGHT),
  rad_bl_drive(&can3_raw, RAD__DRIVE__BACK_LEFT),
  rad_br_drive(&can4_raw, RAD__DRIVE__BACK_RIGHT)
{
  this->declare_parameter("can_rate", 10);
  delay_sec = (1.0 / (4.0 * (float)this->get_parameter("can_rate").as_int()));
  can_pub_ = this->create_publisher<CANstamped>("/can/can_out_queue", 10);

  sub_ = this->create_subscription<SwerveModulesList>(
    "/drive/modules_command", 10, std::bind(&RAD_Drive_Controller::_callback, this, _1)
  );

  rad_fl_drive.set_pid_angle_offset(120.0);
  rad_fr_drive.set_pid_angle_offset(120.0);
  rad_bl_drive.set_pid_angle_offset(120.0);
  rad_br_drive.set_pid_angle_offset(120.0);

  // rad_fl_drive.set_mul_factor(RAD__DRIVE__GEAR_RATIO);
  // rad_fr_drive.set_mul_factor(RAD__DRIVE__GEAR_RATIO);
  // rad_bl_drive.set_mul_factor(RAD__DRIVE__GEAR_RATIO);
  // rad_br_drive.set_mul_factor(RAD__DRIVE__GEAR_RATIO);
}

void RAD_Drive_Controller::_publish_to_can()
{
  CANstamped can1, can2, can3, can4;
  can1.stamp = this->now() + rclcpp::Duration::from_seconds(delay_sec * 0);
  can2.stamp = this->now() + rclcpp::Duration::from_seconds(delay_sec * 1);
  can3.stamp = this->now() + rclcpp::Duration::from_seconds(delay_sec * 2);
  can4.stamp = this->now() + rclcpp::Duration::from_seconds(delay_sec * 3);
  can1.can_raw = can1_raw;
  can2.can_raw = can2_raw;
  can3.can_raw = can3_raw;
  can4.can_raw = can4_raw;

  can_pub_->publish(can1);
  can_pub_->publish(can2);
  can_pub_->publish(can3);
  can_pub_->publish(can4);

}

void RAD_Drive_Controller::_callback(const SwerveModulesList& msg)
{
  rad_fl_drive.set_target_angle(msg.front_left.angle);
  rad_fr_drive.set_target_angle(msg.front_right.angle);
  rad_bl_drive.set_target_angle(msg.rear_left.angle);
  rad_br_drive.set_target_angle(msg.rear_right.angle);
  this->_publish_to_can();
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RAD_Drive_Controller>());
  rclcpp::shutdown();

  // printf("hello world rad_control package\n");

  return 0;
}
