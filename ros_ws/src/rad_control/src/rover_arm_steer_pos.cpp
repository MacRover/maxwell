#include "rad_control/rover_arm_steer_pos.hpp"

ArmSteerPos::ArmSteerPos() : Node("rad_arm_controller"), 
    rate(std::chrono::milliseconds(33)),
    rad_base(&can_msg_base, RAD__ARM__BASE),
    rad_shoulder(&can_msg_shoulder, RAD__ARM__SHOULDER),
    rad_elbow(&can_msg_elbow, RAD__ARM__ELBOW)
{
    can_pub_ = this->create_publisher<CANraw>("/can/can_out", 10);
    sub_ = this->create_subscription<JointTrajectory>(
        "/arm_controller/joint_trajectory", 10, 
        std::bind(&ArmSteerPos::_callback, this, _1)
    );
    timer_ = this->create_wall_timer(
      100ms, std::bind(&ArmSteerPos::_timer_callback, this));
    
    rad_base.set_stepper_speed(0.0);
    rad_shoulder.set_stepper_speed(0.0);
    rad_elbow.set_stepper_speed(0.0);
}

void ArmSteerPos::_callback(const JointTrajectory& msg)
{
    rad_base.set_stepper_speed(msg.points[0].velocities[0]);
    rad_shoulder.set_stepper_speed(msg.points[0].velocities[1]);
    rad_elbow.set_stepper_speed(msg.points[0].velocities[2]);
}

void ArmSteerPos::_timer_callback(void)
{
    can_pub_->publish(can_msg_base);
    rate.sleep();
    can_pub_->publish(can_msg_shoulder);
    rate.sleep();
    can_pub_->publish(can_msg_elbow);
}


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmSteerPos>());
  rclcpp::shutdown();

  return 0;
}
