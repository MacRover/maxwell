#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <memory.h>
#include <unistd.h>
#include <vector>


#define KEYCODE_RA 0x43 
#define KEYCODE_LA 0x44
#define KEYCODE_UA 0x41
#define KEYCODE_DA 0x42

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64

#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_Z 0x7A
#define KEYCODE_X 0x78
#define KEYCODE_R 0x72
#define KEYCODE_F 0x66

#define base_joint_vel 200
#define pitch_joint_vel 200
#define shoulder_joint_vel 200
#define elbow_joint_vel 150
#define wrist_joint_vel 200
#define gripper_joint_vel 200

class KeyboardController
{
public:
  KeyboardController(std::shared_ptr<rclcpp::Node> nh);
  void keyLoop();

private:
  
  std::shared_ptr<rclcpp::Node> nh_;
  std::vector<double> joints_;
  trajectory_msgs::msg::JointTrajectory joint_trajectory_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
  
};

KeyboardController::KeyboardController(std::shared_ptr<rclcpp::Node> nh):
  nh_(nh),
  joints_(6, 0.0)
{
  joint_trajectory_.joint_names.resize(5);
  joint_trajectory_.joint_names = { "arm_base_joint", "arm_shoulder_joint", "arm_elbow_joint", "arm_wrist_joint", "gripper_joint"};
  joint_trajectory_.points.resize(1);
  // joint_trajectory_.points.push_back(trajectory_msgs::msg::JointTrajectoryPoint());
  // joint_trajectory_.points[0].positions = {0.0, 0.0, 0.0, 0.0, 0.0};
  joint_trajectory_.points[0].time_from_start = rclcpp::Duration(1, 0);
  joint_pub_ = nh_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
//   ros::shutdown();
  rclcpp::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
//   ros::init(argc, argv, "teleop_turtle");
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("arm_keyboard_controller");
  KeyboardController teleop_turtle(node);

  signal(SIGINT,quit);

  teleop_turtle.keyLoop();
  
  return(0);
}


void KeyboardController::keyLoop()
{
  char c;
  bool dirty=false;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("");


  for(;;)
  {
    // get the next event from the keyboard  
    if(::read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
    joint_trajectory_.points[0].velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // ROS_DEBUG("value: 0x%02X\n", c);
    switch(c)
    {
      case KEYCODE_Q:
        std::cout << "BASE LEFT" << std::endl;
        joint_trajectory_.points[0].velocities[0] = -base_joint_vel;
        dirty = true;
        break;
      case KEYCODE_E:
        std::cout << "BASE RIGHT" << std::endl;
        joint_trajectory_.points[0].velocities[0] = base_joint_vel;
        dirty = true;
        break;
      case KEYCODE_D: // gripper close
        std::cout << "GRIPPER CLOSE" << std::endl;
        joint_trajectory_.points[0].velocities[1] = gripper_joint_vel;
        dirty = true;
        break;
      case KEYCODE_A:
        std::cout << "GRIPPER OPEN" << std::endl; // DOWN is gripper open
        joint_trajectory_.points[0].velocities[1] = -gripper_joint_vel;
        dirty = true;
        break;
      case KEYCODE_R: // shoulder up
        std::cout << "SHOULDER UP" << std::endl;
        joint_trajectory_.points[0].velocities[2] = shoulder_joint_vel;
        dirty = true;
        break;
      case KEYCODE_F: // shoulder down
        std::cout << "SHOULDER DOWN" << std::endl;
        joint_trajectory_.points[0].velocities[2] = -shoulder_joint_vel;
        dirty = true;
        break;
      case KEYCODE_S: // Gripper down
        std::cout << "GRIPPER DOWN" << std::endl;
        joint_trajectory_.points[0].velocities[3] = elbow_joint_vel;
        dirty = true;
        break;
      case KEYCODE_W:// Gripper up
        std::cout << "GRIPPER UP" << std::endl;
        joint_trajectory_.points[0].velocities[3] = -elbow_joint_vel;
        dirty = true;
        break;
      case KEYCODE_DA:// Pitch down
        std::cout << "PITCH DOWN" << std::endl;
        joint_trajectory_.points[0].velocities[4] = pitch_joint_vel;
        dirty = true;
        break;
      case KEYCODE_UA:// Pitch up
        std::cout << "PITCH UP" << std::endl;
        joint_trajectory_.points[0].velocities[4] = -pitch_joint_vel;
        dirty = true;
        break;
      case KEYCODE_LA: // base left
        std::cout << "BASE LEFT" << std::endl;
        joint_trajectory_.points[0].velocities[5] = base_joint_vel;
        dirty = true;
        break;
      case KEYCODE_RA: // base right
        std::cout << "BASE RIGHT" << std::endl;
        joint_trajectory_.points[0].velocities[5] = -base_joint_vel;
        dirty = true;
        break;
      default:
        std::cout << "STOPPING" << std::endl;
        joint_trajectory_.points[0].velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        dirty = true;
    }
   

    // geometry_msgs::msg::Twist twist;
    // twist.angular.z = a_scale_*angular_;
    // twist.linear.x = l_scale_*linear_;


    if(dirty ==true)
    {
      joint_pub_->publish(joint_trajectory_);    
      dirty=false;
    }
  }


  return;
}


