#include <cstdio>
#include <chrono>
#include "rad_control/rad_drive_controller.hpp"

using std::placeholders::_1;

RAD_Drive_Controller::RAD_Drive_Controller() : Node("rad_drive_controller")
{
  this->declare_parameter("can_rate", 10);
  sleep_msec = 1000 / (4.0 * (float)this->get_parameter("can_rate").as_int());
  can_pub_ = this->create_publisher<CANraw>("/can/can_out", 10);
  sub_ = this->create_subscription<SwerveModulesList>(
    "/modules_command", 10, std::bind(&RAD_Drive_Controller::_callback, this, _1)
  );
}

void RAD_Drive_Controller::_callback(const SwerveModulesList& msg)
{
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
