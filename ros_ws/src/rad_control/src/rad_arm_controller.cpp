#include <cstdio>
#include <chrono>
#include <cmath> 
#include "rad_control/rad_arm_controller.hpp"
using std::placeholders::_1;

RAD_Arm_Controller::RAD_Arm_Controller() : 
  Node("rad_arm_controller"),
  rad_base_arm(&can_base, RAD__ARM__BASE), 
  rad_ls_arm(&can_ls, RAD__LS),
  rad_gripper_arm(&can_gripper, RAD__ARM__GRIPPER),
  rad_shoulder_arm(&can_shoulder, RAD__ARM__SHOULDER),
  rad_elbow_arm(&can_elbow, RAD__ARM__ELBOW),
  rad_rs_arm(&can_rs, RAD__RS)
 
{
  this->declare_parameter("lmin_shoulder", 0.33);
  this->declare_parameter("lmin_elbow", 0.33);
  this->declare_parameter("a_shoulder", 0.24);
  this->declare_parameter("b_shoulder", 0.23);
  this->declare_parameter("a_elbow", 0.216);
  this->declare_parameter("b_elbow", 0.175);
  this->declare_parameter("shoulder_offset", 89.17);
  this->declare_parameter("elbow_offset", 114.724); 
  this->declare_parameter("base_gear_reduction", 0.0);
  this->declare_parameter("screw_max", 13320); 
  this->declare_parameter("can_rate", 10);
  sleep_msec = (uint16_t)(1000.0 / (4.0 * (float)this->get_parameter("can_rate").as_int()));
  can_pub_ = this->create_publisher<CANraw>("/can/can_out", 10);
  sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/arm/hardware/joint_states", 10, std::bind(&RAD_Arm_Controller::_callback, this, _1)
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
    can_pub_->publish(can_ls);
    rate.sleep();
    can_pub_->publish(can_gripper);
    rate.sleep();
    can_pub_->publish(can_shoulder);
    rate.sleep();
    can_pub_->publish(can_elbow);
    rate.sleep();
    can_pub_->publish(can_rs);
}
void RAD_Arm_Controller::_callback(const sensor_msgs::msg::JointState& msg)
{
  shoulder_offset = this->get_parameter("shoulder_offset").as_double();
  elbow_offset = this->get_parameter("elbow_offset").as_double(); 
  lmin_shoulder = this->get_parameter("lmin_shoulder").as_double(); 
  lmin_elbow = this->get_parameter("lmin_elbow").as_double();
  a_shoulder = this->get_parameter("a_shoulder").as_double();
  a_elbow = this->get_parameter("a_elbow").as_double();
  b_shoulder = this->get_parameter("b_shoulder").as_double();
  b_elbow = this->get_parameter("b_elbow").as_double();
  // // screw max parameter add

  float base_angle = msg.position[0]; 
  float shoulder_angle = msg.position[1]*180/M_PI + shoulder_offset;
  float elbow_angle = msg.position[2] + elbow_offset;
  float pitch_angle = msg.position[3];
  float wrist_angle = msg.position[4];
  float gripper_angle = msg.position[5]; 

  float ls = pitch_angle + wrist_angle; 
  float rs = pitch_angle - wrist_angle; 
  float pi = 3.141592653;

  // // Pretty sure isolating for Theta_M gives: 
  // // Possibly needed in degrees depending on movei 8280, fr
   float theta_m_shoulder = 13320 - (360/0.00254)*(-(lmin_shoulder)+sqrt(-2*a_shoulder*b_shoulder*cos(shoulder_angle*pi/180)+pow(a_shoulder,2)+pow(b_shoulder,2)));
   float theta_m_elbow = (360/0.00254)*(-(lmin_elbow)+sqrt(-2*a_elbow*b_elbow*cos(elbow_angle*pi/180)+pow(a_elbow,2)+pow(b_elbow,2)));
 
/*
  SHOULDER
  lmin = 0.33m 
  lmax = 0.425m
  side a = 0.24m
  side b = 0.23m 

  ELBOW
  lmin = 0.33m
  lmax 0.395m 
  side a = 0.216m 
  side b = 0.175m 

  theta_s = acos((a^2 + b^2-L^2)/2*a*b)


*/
  rad_base_arm.set_target_angle(base_angle);
  rad_shoulder_arm.set_target_angle(theta_m_shoulder);
  rad_elbow_arm.set_target_angle(theta_m_elbow);
  rad_ls_arm.set_target_angle(ls);
  rad_rs_arm.set_target_angle(rs);
  rad_gripper_arm.set_target_angle(gripper_angle);

  std::cout<< theta_m_shoulder << std::endl; 
  //std::cout<< theta_m_elbow << std::endl;
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
