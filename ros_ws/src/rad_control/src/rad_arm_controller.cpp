#include <cstdio>
#include <chrono>
#include "rad_control/rad_arm_controller.hpp"
using std::placeholders::_1;

RAD_Arm_Controller::RAD_Arm_Controller() : 
  Node("rad_arm_controller"),
  rad_base_arm(&can_base, RAD__ARM__BASE), 
  rad_pitch_arm(&can_pitch, RAD__ARM__PITCH),
  rad_gripper_arm(&can_gripper, RAD__ARM__GRIPPER),
  rad_shoulder_arm(&can_shoulder, RAD__ARM__SHOULDER),
  rad_elbow_arm(&can_elbow, RAD__ARM__ELBOW),
  rad_wrist_arm(&can_wrist, RAD__ARM__WRIST)
{
  this->declare_parameter("can_rate", 10);
  sleep_msec = (uint16_t)(1000.0 / (4.0 * (float)this->get_parameter("can_rate").as_int()));
  can_pub_ = this->create_publisher<CANraw>("/can/can_out", 10);
  std::cout<<"H";
  sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, std::bind(&RAD_Arm_Controller::_callback, this, _1)
  );

 /* pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/arm/can/can_out", 10
  ); */

  /*timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&RAD_Arm_Controller::_timer_callback, this) 
  );*/
}
void RAD_Arm_Controller::_publish_to_can()
{
   rclcpp::Rate rate{std::chrono::milliseconds(sleep_msec)};

    can_pub_->publish(can_base);
    rate.sleep();
    can_pub_->publish(can_pitch);
    rate.sleep();
    can_pub_->publish(can_gripper);
    rate.sleep();
    can_pub_->publish(can_shoulder);
    rate.sleep();
    can_pub_->publish(can_elbow);
    rate.sleep();
    can_pub_->publish(can_wrist);
}
void RAD_Arm_Controller::_callback(const sensor_msgs::msg::JointState& msg)
{
  float base_angle = msg.position[0]; 
  float  shoulder_angle = msg.position[1];
  float elbow_angle = msg.position[2];
  float ls = msg.position[3];
  float rs = msg.position[4];
  float gripper_angle = msg.position[5]; 

  //float theta_c = ((L_max - L_min)*360)/2.55; //Modifier for the output angles

  float pitch_angle = (ls + rs)/2;
  float wrist_angle = (ls - rs)/2; 
  
  // Pretty sure isolating for Theta_M gives: 
  // (-360/2.54)*(-L_min-a^2-b^2+sqrt(cos_inv*2*a*b*(theta_s+theta_1-theta_2))) 
  // theta_m, max = ((L_max - L_min)*360)/2.54 

  rad_base_arm.set_target_angle(base_angle);
  rad_shoulder_arm.set_target_angle(shoulder_angle);
  rad_elbow_arm.set_target_angle(elbow_angle);
  rad_pitch_arm.set_target_angle(pitch_angle);
  rad_wrist_arm.set_target_angle(wrist_angle);
  rad_gripper_arm.set_target_angle(gripper_angle);

  this->_publish_to_can(); 
}


/*void RAD_Arm_Controller::_timer_callback(void)
{
    can_pub_->publish(can_base);
    can_pub_->publish(can_pitch);
    can_pub_->publish(can_shoulder);
    can_pub_->publish(can_elbow);
    can_pub_->publish(can_wrist);
    can_pub_->publish(can_gripper);
}*/
int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RAD_Arm_Controller>());
  rclcpp::shutdown();
  return 0;
}
