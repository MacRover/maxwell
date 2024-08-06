#include <cstdio>
#include <chrono>
#include "rad_control/rad_drive_controller.hpp"

using std::placeholders::_1;

RAD_Drive_Controller::RAD_Drive_Controller() : 
  Node("rad_drive_controller"),
  rad_fl_drive(&can1, RAD__DRIVE__FRONT_LEFT),
  rad_fr_drive(&can2, RAD__DRIVE__FRONT_RIGHT),
  rad_bl_drive(&can3, RAD__DRIVE__BACK_LEFT),
  rad_br_drive(&can4, RAD__DRIVE__BACK_RIGHT)
{
  this->declare_parameter("can_rate", 10);
  sleep_msec = (uint16_t)(1000.0 / (4.0 * (float)this->get_parameter("can_rate").as_int()));
  can_pub_ = this->create_publisher<CANraw>("/can/can_out", 10);

  sub_ = this->create_subscription<SwerveModulesList>(
    "/modules_command", 10, std::bind(&RAD_Drive_Controller::_callback, this, _1)
  );
}

void RAD_Drive_Controller::_publish_to_can()
{
  rclcpp::Rate rate{std::chrono::milliseconds(sleep_msec)};

  can_pub_->publish(can1);
  rate.sleep();
  can_pub_->publish(can2);
  rate.sleep();
  can_pub_->publish(can3);
  rate.sleep();
  can_pub_->publish(can4);

}

void RAD_Drive_Controller::_callback(const SwerveModulesList& msg)
{
  rad_fl_drive.set_target_angle((float)msg.front_left.angle);
  rad_fr_drive.set_target_angle((float)msg.front_right.angle);
  rad_bl_drive.set_target_angle((float)msg.rear_left.angle);
  rad_br_drive.set_target_angle((float)msg.rear_right.angle);
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
