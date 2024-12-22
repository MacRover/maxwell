#include <cstdio>
#include <chrono>
#include "rad_control/rover_steer_pos.hpp"

SteerPos::SteerPos(std::string name) : Node(name), 
    rad_fr(&can_msg_1, RAD__DRIVE__FRONT_RIGHT), 
    rad_fl(&can_msg_2, RAD__DRIVE__FRONT_LEFT),
    rad_br(&can_msg_3, RAD__DRIVE__BACK_RIGHT),
    rad_bl(&can_msg_4, RAD__DRIVE__BACK_LEFT)
{
    rad_fr.pulse_stepper(0);
    rad_fl.pulse_stepper(0);
    rad_br.pulse_stepper(0);
    rad_bl.pulse_stepper(0);

    this->declare_parameter("can_rate", 10);
    sleep_msec = (uint16_t)(1000.0 / (4.0 * (float)this->get_parameter("can_rate").as_int()));

    can_pub_ = this->create_publisher<CANraw>("/can/can_out", 10);
    sub_ = this->create_subscription<SwerveModulePulse>(
        "/rad_pulses_repeat", 10, std::bind(&SteerPos::_pulse_callback, this, _1)
    );
}

void SteerPos::_pulse_callback(const SwerveModulePulse& msg)
{
  rad_fr.pulse_stepper(msg.front_right_pulse);
  rad_fl.pulse_stepper(msg.front_left_pulse);
  rad_br.pulse_stepper(msg.rear_right_pulse);
  rad_bl.pulse_stepper(msg.rear_left_pulse);
  this->_publish_to_can();
}

void SteerPos::_publish_to_can()
{
  rclcpp::Rate rate{std::chrono::milliseconds(sleep_msec)};
  can_pub_->publish(can_msg_1);
  rate.sleep();
  can_pub_->publish(can_msg_2);
  rate.sleep();
  can_pub_->publish(can_msg_3);
  rate.sleep();
  can_pub_->publish(can_msg_4);
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SteerPos>("steer_node"));
  rclcpp::shutdown();

  return 0;
}
