#include "rad_control/rover_steer_pid.hpp"

SteerTestPID::SteerTestPID() : Node("steer_pid_node"), 
rad_fl(&can_fr, RAD_STEER_PID__FRONT_LEFT),
rad_fr(&can_fl, RAD_STEER_PID__FRONT_RIGHT)
{
    can_pub_ = this->create_publisher<CANraw>("/can/can_out", 10);
    sub_ = this->create_subscription<Float32>(
        "/test/rad_speed", 10, std::bind(&SteerTestPID::_callback, this, _1)
    );
    timer_ = this->create_wall_timer(
      200ms, std::bind(&SteerTestPID::_timer_callback, this));
    
    rad_fl.set_pid_angle_offset(0.0);
    rad_fr.set_pid_angle_offset(0.0);
    rad_fl.set_mul_factor(RAD__DRIVE__GEAR_RATIO);
    rad_fr.set_mul_factor(RAD__DRIVE__GEAR_RATIO);
}

void SteerTestPID::_callback(const Float32& msg)
{
    rad_fl.set_target_angle(msg.data);
    rad_fr.set_target_angle(msg.data);
}

void SteerTestPID::_timer_callback()
{
    can_pub_->publish(can_fr);
    can_pub_->publish(can_fl);
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SteerTestPID>());
  rclcpp::shutdown();

  return 0;
}