#include "rad_control/rad_calibration_init.hpp"
#include <chrono>
#include <thread>

using std::placeholders::_1;

RAD_Init::RAD_Init() : Node("rad_init"), fr_ls(false), fl_ls(false), br_ls(false), bl_ls(false)
{
    sub_fr = this->create_subscription<RadStatus>(
    "/front_right/rad_status", 10, std::bind(&RAD_Init::_callback_fr, this, _1)
  );
    sub_fl = this->create_subscription<RadStatus>(
    "/front_left/rad_status", 10, std::bind(&RAD_Init::_callback_fl, this, _1)
  );
    sub_br = this->create_subscription<RadStatus>(
    "/rear_right/rad_status", 10, std::bind(&RAD_Init::_callback_br, this, _1)
  );
    sub_bl = this->create_subscription<RadStatus>(
    "/rear_left/rad_status", 10, std::bind(&RAD_Init::_callback_bl, this, _1)
  );
}

bool RAD_Init::finished()
{
    return this->fr_ls && this->fl_ls && this->br_ls && this->bl_ls;
}

void RAD_Init::_callback_fr(const RadStatus& msg)
{
    this->fr_ls = msg.ls_state;
}
void RAD_Init::_callback_fl(const RadStatus& msg)
{
    this->fl_ls = msg.ls_state;
}
void RAD_Init::_callback_br(const RadStatus& msg)
{
    this->br_ls = msg.ls_state;
}
void RAD_Init::_callback_bl(const RadStatus& msg)
{
    this->bl_ls = msg.ls_state;
}

std::shared_ptr<rclcpp::Publisher<CANraw>> can_pub;
std::shared_ptr<RAD_Init> init_node;

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  rclcpp::init(argc, argv);

  CANraw can1, can2, can3, can4;

  RAD rad_fl_drive{&can1, RAD__DRIVE__FRONT_LEFT}, 
      rad_fr_drive{&can2, RAD__DRIVE__FRONT_RIGHT}, 
      rad_bl_drive{&can3, RAD__DRIVE__BACK_LEFT}, 
      rad_br_drive{&can4, RAD__DRIVE__BACK_RIGHT};

  init_node = std::make_shared<RAD_Init>();
  std::thread spin_thread([](){rclcpp::spin(init_node);});

  can_pub = init_node->create_publisher<CANraw>("/can/can_out", 10);
  rclcpp::Rate rate{std::chrono::milliseconds(1000)};

  RCLCPP_INFO(init_node->get_logger(), "Zeroing RAD drive motors");
  
  // Constantly rotate motors until it hits limit switch
  // Quit once all motors have been calibrated
  while (!init_node->finished() && rclcpp::ok())
  {
    rad_fl_drive.calibrate_zero_pos();
    rad_fr_drive.calibrate_zero_pos();
    rad_bl_drive.calibrate_zero_pos();
    rad_br_drive.calibrate_zero_pos();
    can_pub->publish(can1);
    can_pub->publish(can2);
    can_pub->publish(can3);
    can_pub->publish(can4);
    rate.sleep();
  }

  RCLCPP_INFO(init_node->get_logger(), "Success!");

  // kill node and spin thread
  spin_thread.~thread();
  rclcpp::shutdown();

  return 0;
}