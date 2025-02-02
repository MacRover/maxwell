#include <cstdio>
#include <chrono>
#include "arm_controller/RAD_Arm_Controller.hpp"

using std::placeholders::_1;

RAD_Arm_Controller::RAD_Arm_Controller() : 
  Node("rad_arm_controller"),
  rad_base_arm(&can_msg_base, RAD__ARM__BASE), // For the drive_control, they have these can ids already filled in rad.hpp. (RAD__ARM__BASE)
  rad_pitch_arm(&can_msg_pitch, RAD__ARM__PITCH),
  rad_gripper_arm(&can_msg_gripper, RAD__ARM__GRIPPER),
  rad_shoulder_arm(&can_msg_shoulder, RAD__ARM__SHOULDER),
  rad_elbow_arm(&can_msg_elbow, RAD_ARM_ELBOW),
  rad_wrist_arm(&can_msg_wrist, RAD_ARM_WRIST)
{
  this->declare_parameter("can_rate", 10);
  sleep_msec = (uint16_t)(1000.0 / (4.0 * (float)this->get_parameter("can_rate").as_int())); 
  can_pub_ = this->create_publisher<CANraw>("/can/can_out", 10); //Can I use this instead of CAN writer node? 

  sub_ = this->create_subscription<JointState>(
    "/modules_command", 10, std::bind(&RAD_Arm_Controller::_callback, this, _1)
  );
}

void RAD_Arm_Controller::_publish_to_can()
{
  rclcpp::Rate rate{std::chrono::milliseconds(sleep_msec)};

  can_pub_->publish(can_msg_base);
  rate.sleep();
  can_pub_->publish(can_msg_elbow);
  rate.sleep();
  can_pub_->publish(can_msg_shoulder);
  rate.sleep();
  can_pub_->publish(can_msg_wrist);
  rate.sleep();  
  can_pub_->publish(can_msg_gripper);
  rate.sleep();
  can_pub_->publish(can_msg_pitch);
  rate.sleep();    
}

void RAD_Arm_Controller::_callback(const JointState& msg)
{
    //Note: these are not the correct positions for these joints
  double base_angle = (msg.position[0]);
  double shoulder_angle = (msg.position[1]);
  double elbow_angle = (msg.position[2]); 
  double gripper_angle = (msg.position[5]); 

    /*
        WRIST LOGIC!!!
        //Assuming that the left encoder is msg.position[3]; 

        double ls = (msg.position[3]); 
        double rs = (msg.position[4]);

        double pitch_angle = (ls + rs)/2; 
        double wrist_angle = (ls - rs)/2; 
        
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
  rad_elbow_angle.set_target_angle(elbow_angle);
  rad_pitch_angle.set_target_angle(pitch_angle);
  rad_wrist_angle.set_target_angle(wrist_angle);
  rad_gripper_angle.set_target_angle(gripper_angle);
  this->_publish_to_can();
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
