#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/ca_nraw.hpp"
#include "rad_control/rad.hpp"

using namespace custom_interfaces::msg;
using std::placeholders::_1;

std::shared_ptr<rclcpp::Node> can_config;
std::shared_ptr<rclcpp::Publisher<CANraw>> can_pub;
std::shared_ptr<rclcpp::Subscription<CANraw>> can_sub;
uint8_t rad_id, command_id;
bool ack;

std::map<std::string, uint8_t> get_cmd = {
  {"GET_TARGET_ANGLE", CAN_GET_TARGET_ANGLE},
  {"GET_P_VALUE", CAN_GET_P_VALUE},
  {"GET_I_VALUE", CAN_GET_I_VALUE},
  {"GET_D_VALUE", CAN_GET_D_VALUE}
};
std::map<std::string, uint8_t> set_cmd = {
  {"SET_TARGET_ANGLE", CAN_SET_TARGET_ANGLE},
  {"SET_P_VALUE", CAN_SET_P_VALUE},
  {"SET_I_VALUE", CAN_SET_I_VALUE},
  {"SET_D_VALUE", CAN_SET_D_VALUE}
};


void response(const CANraw& msg)
{
  if (((msg.address & 0xff) == rad_id) && (((msg.address >> 8) & 0xff) == command_id))
  {
    RCLCPP_INFO(can_config->get_logger(), "Message received!");
    ack = true;
  }
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  
  can_config = std::make_shared<rclcpp::Node>("rad_tool_node");
  can_pub = can_config->create_publisher<CANraw>("/can/can_out", 10);
  can_sub = can_config->create_subscription<CANraw>("/can/config/rad_can_in", 10, response);
  CANraw can_out_msg;

  std::thread spin_thread([](){rclcpp::spin(can_config);});

  std::string in;

  while(true)
  {
    ack = false;
    std::cout << "Enter RAD ID (q to exit): ";
    std::getline (std::cin,in);
    if (in == "q")
      break;
    rad_id = std::stoi(in);

    do {
      std::cout << "Enter Command ('?' to list options): ";
      std::getline (std::cin,in);
      if (in == "?")
      {
        for (const auto&[command_name, _] : get_cmd)
          std::cout << command_name << std::endl;
        for (const auto&[command_name, _] : set_cmd)
          std::cout << command_name << std::endl;
      }
    }
    while (in == "?");

    command_id = ((get_cmd.count(in) == 1) ? get_cmd[in] : set_cmd[in]);
    can_out_msg.address = (2 << 25) | (rad_id) | (command_id << 8);
    can_out_msg.data = {0};
    uint8_t i = 0;

    if (set_cmd.count(in) == 1)
    {
      std::string val_in;
      std::cout << "Enter value: ";
      std::getline (std::cin,val_in);
      if (command_id == CAN_SET_TARGET_ANGLE)
      {
        can_out_msg.data.resize(8, 0);
        __buffer_append_float64(&(can_out_msg.data[0]), std::stod(val_in), &i);
      }
      else if (command_id == CAN_SET_P_VALUE || command_id == CAN_SET_I_VALUE || command_id == CAN_SET_D_VALUE)
      {
        can_out_msg.data.resize(4, 0);
        __buffer_append_float32(&(can_out_msg.data[0]), std::stof(val_in), &i);
      }
      else 
      {
        can_out_msg.data[0] = (uint8_t)std::stoi(val_in);
      }
    }

    RCLCPP_INFO(can_config->get_logger(), "RAD_ID: %d, COMMAND_ID: %d", rad_id, command_id);
    RCLCPP_INFO(can_config->get_logger(), "SENT CAN FRAME %x", can_out_msg.address);
    can_pub->publish(can_out_msg);

    if (get_cmd.count(in) == 1) while (!ack){};
  }
  
  spin_thread.~thread();
  rclcpp::shutdown();
  
  return 0;
}