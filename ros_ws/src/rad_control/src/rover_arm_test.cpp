#include "rad_control/rover_arm_test.hpp"

ArmTestRADController::ArmTestRADController() : Node("rad_arm_controller"), 
    rate(std::chrono::milliseconds(33))
{
    can_pub_ = this->create_publisher<CANraw>("/can/can_out", 10);
    sub_ = this->create_subscription<JointTrajectory>(
        "/arm_controller/joint_trajectory", 10, 
        std::bind(&ArmTestRADController::_callback, this, _1)
    );
    timer_ = this->create_wall_timer(
      200ms, std::bind(&ArmTestRADController::_timer_callback, this));
    
    can_msg_base.address = 0x154;
    can_msg_pitch.address = 0x1A54;
    can_msg_shoulder.address = 0x254;
    can_msg_elbow.address = 0x1B54;
    can_msg_wrist.address = 0x1854;
    can_msg_gripper.address = 0x1954;

    can_msg_base.data = {0,0,0,0,0,0,0,0};
    can_msg_pitch.data = {0,0,0,0,0,0,0,0};
    can_msg_shoulder.data = {0,0,0,0,0,0,0,0};
    can_msg_elbow.data = {0,0,0,0,0,0,0,0};
    can_msg_wrist.data = {0,0,0,0,0,0,0,0};
    can_msg_gripper.data = {0,0,0,0,0,0,0,0};
}

void ArmTestRADController::_callback(const JointTrajectory& msg)
{
    uint8_t base_dir, pitch_dir, shoulder_dir, elbow_dir, wrist_dir, gripper_dir;

    base_dir = (msg.points[0].velocities[0] > 0.0) ? 0x54 : 0x55;
    pitch_dir = (msg.points[0].velocities[1] > 0.0) ? 0x54 : 0x55;
    shoulder_dir = (msg.points[0].velocities[2] > 0.0) ? 0x54 : 0x55;
    elbow_dir = (msg.points[0].velocities[3] > 0.0) ? 0x54 : 0x55;
    wrist_dir = (msg.points[0].velocities[4] > 0.0) ? 0x54 : 0x55;
    gripper_dir = (msg.points[0].velocities[5] > 0.0) ? 0x54 : 0x55;
    
    // Set speed range: [-255, 255]
    can_msg_base.address = (0x00000001 << 8 | base_dir);
    can_msg_base.data[7] = (uint8_t)std::abs(msg.points[0].velocities[0]);
    can_msg_pitch.address = (0x0000001A << 8 | pitch_dir);
    can_msg_pitch.data[7] = (uint8_t)std::abs(msg.points[0].velocities[1]);
    can_msg_shoulder.address = (0x00000002 << 8 | shoulder_dir);
    can_msg_shoulder.data[7] = (uint8_t)std::abs(msg.points[0].velocities[2]);
    can_msg_elbow.address = (0x0000001B << 8 | elbow_dir);
    can_msg_elbow.data[7] = (uint8_t)std::abs(msg.points[0].velocities[3]);
    can_msg_wrist.address = (0x00000018 << 8 | wrist_dir);
    can_msg_wrist.data[7] = (uint8_t)std::abs(msg.points[0].velocities[4]);
    can_msg_gripper.address = (0x00000019 << 8 | gripper_dir);
    can_msg_gripper.data[7] = (uint8_t)std::abs(msg.points[0].velocities[5]);
}

void ArmTestRADController::_timer_callback(void)
{
    can_pub_->publish(can_msg_base);
    rate.sleep();
    can_pub_->publish(can_msg_pitch);
    rate.sleep();
    can_pub_->publish(can_msg_shoulder);
    rate.sleep();
    can_pub_->publish(can_msg_elbow);
    rate.sleep();
    can_pub_->publish(can_msg_wrist);
    rate.sleep();
    can_pub_->publish(can_msg_gripper);
}


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmTestRADController>());
  rclcpp::shutdown();

  return 0;
}
