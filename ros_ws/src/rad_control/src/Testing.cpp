#include <cstdio>
#include <chrono>
#include "rad_control/RAD_Arm_Controller.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


using std::placeholders::_1;

RAD_Arm_Controller::RAD_Arm_Controller() : 
  Node("rad_arm_controller"),
  rad_base_arm(RAD__ARM__BASE), //Does RAD__ARM__BASE, etc need can ID's? 
  rad_pitch_arm(RAD__ARM__PITCH),
  rad_gripper_arm(RAD__ARM__GRIPPER),
  rad_shoulder_arm(RAD__ARM__SHOULDER),
  rad_elbow_arm(RAD__ARM__ELBOW),
  rad_wrist_arm(RAD__ARM__WRIST)
{
  sub_ = this->create_subscription<JointState>(
    "/modules_command", 10, std::bind(&RAD_Arm_Controller::_callback, this, _1)
  );

  pub_ = this->create_publisher<JointState>(
    "/arm/can/can_out", 10
  ); 

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&RAD_Arm_Controller::publish_joint_angles, this) 
  );
}
void RAD_Arm_Controller::_callback(const JointState& msg)
{
    //Note: positions may not be properly corresponding
  double base_angle = (msg.position[0]);
  double shoulder_angle = (msg.position[1]);
  double elbow_angle = (msg.position[2]); 
  double ls = (msg.position[3]);
  double rs = (msg.position[4]);
  double gripper_angle = (msg.position[5]); 

  double pitch_angle = (ls + rs)/2;
  double wrist_angle = (ls - rs)/2; 
    /*
        WRIST LOGIC!!!
        //Assuming that the left encoder is msg.position[3];
    if(pitch_limit_swtich == 1)
    {
        if (current_pitch == MAX_PITCH)
        {
            Publish this to a topic, set topic so that if current_pitch == MAX_PITCH, that pitch inputted must be = or < current_pitch, else ignored 
        }
        else
        {
            Publish this to a topic, set topic so that if current_pitch == MIN_PITCH, that pitch inputted must be = or > current_pitch, else ignored
        }
    }

    if(roll_limit_switch == 1)
    {
        if(current_roll == MAX_ROLL)
        {
            Publish this to a topic, set topic so that if current_roll == MAX_ROLL, that roll inputted must be = or < current_roll, else ignored
        }
        else
        {
            Publish this to a topic, set topic so that if current_pitch == MIN_ROLL, that roll inputted must be = or > current_roll, else ignored
        }
    }

    */
  rad_base_arm.set_target_angle(base_angle);
  rad_shoulder_arm.set_target_angle(shoulder_angle);
  rad_elbow_arm.set_target_angle(elbow_angle);
  rad_pitch_arm.set_target_angle(pitch_angle);
  rad_wrist_arm.set_target_angle(wrist_angle);
  rad_gripper_arm.set_target_angle(gripper_angle);

}

void RAD_Arm_Controller::publish_joint_angles(const JointState& msg)
{
  auto new_msg = sensor_msgs::msg::JointState();
  new_msg.name = {"base", "shoulder", "elbow", "pitch", "wrist", "gripper"};
  new_msg.position = {rad_base_arm, rad_shoulder_arm, rad_elbow_arm, rad_pitch_arm, rad_wrist_arm, rad_gripper_arm};
  pub_->publish(new_msg); 
}
int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RAD_Arm_Controller>());
  rclcpp::shutdown();
  return 0;
}
