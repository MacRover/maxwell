#include "rad_control/rover_arm_steer_pos.hpp"

ArmSteerPos::ArmSteerPos() : Node("rad_arm_controller"), 
    rad_base(&can_msg_base.can_raw, RAD__ARM__BASE),
    rad_shoulder(&can_msg_shoulder.can_raw, RAD__ARM__SHOULDER),
    rad_elbow(&can_msg_elbow.can_raw, RAD__ARM__ELBOW),
    rad_wrist_left(&can_msg_wrist_left.can_raw, RAD__ARM__WRIST_LS),
    rad_wrist_right(&can_msg_wrist_right.can_raw, RAD__ARM__WRIST_RS),
    rad_wrist_gripper(&can_msg_wrist_gripper.can_raw, RAD__ARM__GRIPPER)
{
    can_pub_ = this->create_publisher<CANstamped>("/can/can_out_queue", 10);
    sub_ = this->create_subscription<JointTrajectory>(
        "/arm_controller/joint_trajectory", 10, 
        std::bind(&ArmSteerPos::_callback, this, _1)
    );

    rad_base.pulse_stepper(0.0);
    rad_shoulder.pulse_stepper(0.0);
    rad_elbow.pulse_stepper(0.0);
    rad_wrist_left.pulse_stepper(0.0);
    rad_wrist_right.pulse_stepper(0.0);
    rad_wrist_gripper.pulse_stepper(0.0);
    
    timer_ = this->create_wall_timer(
      100ms, std::bind(&ArmSteerPos::_timer_callback, this));
}

void ArmSteerPos::_callback(const JointTrajectory& msg)
{
    double roll = msg.points[0].velocities[3];
    double pitch = msg.points[0].velocities[4];
    rad_base.pulse_stepper(msg.points[0].velocities[0]);
    rad_shoulder.pulse_stepper(msg.points[0].velocities[1]);
    rad_elbow.pulse_stepper(msg.points[0].velocities[2]);
    rad_wrist_left.pulse_stepper(pitch + roll);
    rad_wrist_right.pulse_stepper(pitch - roll);
    rad_wrist_gripper.pulse_stepper(msg.points[0].velocities[5]);
}

void ArmSteerPos::_timer_callback(void)
{
    can_msg_base.stamp = this->now() + rclcpp::Duration::from_seconds(16 / 1000.0f * 0);
    can_msg_shoulder.stamp = this->now() + rclcpp::Duration::from_seconds(16 / 1000.0f * 1);
    can_msg_elbow.stamp = this->now() + rclcpp::Duration::from_seconds(16 / 1000.0f * 2);
    can_msg_wrist_left.stamp = this->now() + rclcpp::Duration::from_seconds(16 / 1000.0f * 3);
    can_msg_wrist_right.stamp = this->now() + rclcpp::Duration::from_seconds(16 / 1000.0f * 4);
    can_msg_wrist_gripper.stamp = this->now() + rclcpp::Duration::from_seconds(16 / 1000.0f * 5);

    can_pub_->publish(can_msg_base);
    can_pub_->publish(can_msg_shoulder);
    can_pub_->publish(can_msg_elbow);
    can_pub_->publish(can_msg_wrist_left);
    can_pub_->publish(can_msg_wrist_right);
    can_pub_->publish(can_msg_wrist_gripper);
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
