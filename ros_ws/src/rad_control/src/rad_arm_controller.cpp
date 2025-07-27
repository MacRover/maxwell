#include <cstdio>
#include <chrono>
#include <cmath> 
#include "rad_control/rad_arm_controller.hpp"
// ROS in
#define LEN_MM_TO_ROTATION (360/0.00254)
using std::placeholders::_1;

RAD_Arm_Controller::RAD_Arm_Controller() : 
  Node("rad_arm_controller"),
  rad_base_arm(&can_base, RAD__ARM__BASE), 
  rad_ls_arm(&can_ls, RAD__ARM__WRIST_LS),
  rad_gripper_arm(&can_gripper, RAD__ARM__GRIPPER),
  rad_shoulder_arm(&can_shoulder, RAD__ARM__SHOULDER),
  rad_elbow_arm(&can_elbow, RAD__ARM__ELBOW),
  rad_rs_arm(&can_rs, RAD__ARM__WRIST_RS)
{
  this->declare_parameter<std::vector<double>>("lmins", {0.33, 0.33});        // Shoulder, elbow
  this->declare_parameter<std::vector<double>>("a_lengths", {0.24, 0.216});   // Shoulder, elbow
  this->declare_parameter<std::vector<double>>("b_lengths", {0.23, 0.175});   // Shoulder, elbow
  this->declare_parameter<std::vector<double>>("offsets", {89.17, 114.724});  // Shoulder, elbow
  this->declare_parameter("screw_max", 13320.0);
  this->declare_parameter("can_rate", 10); 
  this->declare_parameter("gripper_steps", 200);
  lmins = this->get_parameter("lmins").as_double_array();
  a_lengths = this->get_parameter("a_lengths").as_double_array(); 
  b_lengths = this->get_parameter("b_lengths").as_double_array();
  offsets = this->get_parameter("offsets").as_double_array();
  screw_max = this->get_parameter("screw_max").as_double(); 

  gripper_steps = this->get_parameter("gripper_steps").as_double(); 

  sleep_msec = (uint16_t)(1000.0 / (4.0 * (float)this->get_parameter("can_rate").as_int()));

  can_pub_ = this->create_publisher<CANraw>("/can/can_out", 10);

  sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/arm/hardware/joint_states", 10, std::bind(&RAD_Arm_Controller::_callback, this, _1));
  
  gripper_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/arm/finger/joints", 10, std::bind(&RAD_Arm_Controller::_callback_gripper, this, _1));
}
void RAD_Arm_Controller::_publish_to_can()
{
   rclcpp::Rate rate{std::chrono::milliseconds(sleep_msec)};

    can_pub_->publish(can_base);
    rate.sleep();
    can_pub_->publish(can_ls);
    rate.sleep();
    can_pub_->publish(can_shoulder);
    rate.sleep();
    can_pub_->publish(can_elbow);
    rate.sleep();
    can_pub_->publish(can_rs);
}

float RAD_Angle_Conversion(float angle, float lmin, float pi, float a, float b){

  float length = sqrt(-2*a*b*cos(angle*pi/180) + pow(a,2) + pow(b,2)); 
  float theta_m = LEN_MM_TO_ROTATION * (length - lmin); 

  return theta_m; 
}
void RAD_Arm_Controller::_callback(const sensor_msgs::msg::JointState& msg)
{
  float base_angle = msg.position[0]; 
  float shoulder_angle = msg.position[1]*180/M_PI + offsets[0];
  float elbow_angle = msg.position[2]*180/M_PI + offsets[1];
  float pitch_angle = msg.position[3]*180/M_PI;
  float wrist_angle = msg.position[4]*180/M_PI;
  
  float ls = 60 *(pitch_angle + wrist_angle); 
  float rs = 60 *(pitch_angle - wrist_angle); 
  float pi = 3.141592653;

  float theta_m_shoulder = screw_max - RAD_Angle_Conversion(shoulder_angle, lmins[0], pi, a_lengths[0], b_lengths[0]); 
  float theta_m_elbow = 8297 - RAD_Angle_Conversion(elbow_angle, lmins[1], pi, a_lengths[1], b_lengths[1]); 
 
  rad_base_arm.set_target_angle(base_angle);
  rad_shoulder_arm.set_target_angle(theta_m_shoulder);
  rad_elbow_arm.set_target_angle(theta_m_elbow);
  rad_ls_arm.set_target_angle(ls);
  rad_rs_arm.set_target_angle(rs);
  // rad_gripper_arm.set_target_angle(gripper_angle);

  // std::cout << "Shoulder: " << theta_m_shoulder << std::endl;
  // std::cout << "Elbow: " << theta_m_elbow << std::endl;

  //ROS info
  RCLCPP_INFO(this->get_logger(), "Base angle: %f, Shoulder angle: %f, Elbow angle: %f, LS angle: %f, RS angle: %f", 
              base_angle, theta_m_shoulder, theta_m_elbow, ls, rs);
  

  this->_publish_to_can(); 
}
void RAD_Arm_Controller::_callback_gripper(const std_msgs::msg::Int32& msg)
{
  if(msg.data > 0){
    rad_gripper_arm.pulse_stepper(gripper_steps);
  } else if(msg.data < 0){
    rad_gripper_arm.pulse_stepper(-gripper_steps);
  } else {
    rad_gripper_arm.pulse_stepper(0.0);
  }

  rclcpp::Rate rate{std::chrono::milliseconds(sleep_msec)};
  can_pub_->publish(can_gripper);
  rate.sleep();

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

