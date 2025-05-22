#include "rad_control/rad_calibration_wrist.hpp"
#include <chrono>
#include <thread>

using std::placeholders::_1;

RAD_Wrist_Init::RAD_Wrist_Init() : 
    Node("rad_wrist_init"),
    left_ls(false),
    right_ls(false)
{
    this->declare_parameter<std::string>("can_topic", "/can/can_out");
    left_wrist_sub = this->create_subscription<RadStatus>(
        "/arm/wrist_ls/rad_status", 10, std::bind(&RAD_Wrist_Init::_ls_callback, this, _1)
      );
    right_wrist_sub = this->create_subscription<RadStatus>(
        "/arm/wrist_rs/rad_status", 10, std::bind(&RAD_Wrist_Init::_rs_callback, this, _1)
    );
}

void RAD_Wrist_Init::_ls_callback(const RadStatus& msg)
{
    this->left_ls = msg.ls_state_2;
}

void RAD_Wrist_Init::_rs_callback(const RadStatus& msg)
{
    this->right_ls = msg.ls_state_2;
}

bool RAD_Wrist_Init::is_roll_calibrated()
{
    // right wrist for roll calibrate
    return this->right_ls;
}

bool RAD_Wrist_Init::is_pitch_calibrated()
{
    // left wrist for pitch calibrate
    return this->left_ls;
}

std::shared_ptr<rclcpp::Publisher<CANraw>> can_pub;
std::shared_ptr<RAD_Wrist_Init> wrist_init_node;

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  wrist_init_node = std::make_shared<RAD_Wrist_Init>();

  std::thread spin_thread([](){rclcpp::spin(wrist_init_node);});

  std::string can_topic = wrist_init_node->get_parameter("can_topic").as_string();
  can_pub = wrist_init_node->create_publisher<CANraw>(can_topic, 10);
  rclcpp::Rate rate{std::chrono::milliseconds(50)};

  CANraw can1_raw, can2_raw;
  RAD rad_ls(&can1_raw, RAD__ARM__WRIST_LS), 
      rad_rs(&can2_raw, RAD__ARM__WRIST_RS);

  while (!wrist_init_node->is_roll_calibrated() && rclcpp::ok())
  {
    rad_ls.pulse_stepper(-100);
    rad_rs.pulse_stepper(100);
    can_pub->publish(can1_raw);
    can_pub->publish(can2_raw);
    rate.sleep();
  }

  RCLCPP_INFO(wrist_init_node->get_logger(), "Roll calibrated");

  while (!wrist_init_node->is_pitch_calibrated() && rclcpp::ok())
  {
    rad_ls.pulse_stepper(100);
    rad_rs.pulse_stepper(100);
    can_pub->publish(can1_raw);
    can_pub->publish(can2_raw);
    rate.sleep();
  }

  // Zero encoder position
  rad_ls.set_zero_point(); 
  rad_rs.set_zero_point();
  can_pub->publish(can1_raw);
  can_pub->publish(can2_raw);

  RCLCPP_INFO(wrist_init_node->get_logger(), "Pitch calibrated");

  // kill node and spin thread
  spin_thread.~thread();
  rclcpp::shutdown();

}





