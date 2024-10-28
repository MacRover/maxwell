#include <thread>
#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/ca_nraw.hpp"
#include "rad_control/rad.hpp"

using namespace std::chrono_literals;
using namespace custom_interfaces::msg;
using std::placeholders::_1;

std::shared_ptr<rclcpp::Node> can_config;
std::shared_ptr<rclcpp::Publisher<CANraw>> can_pub;
std::shared_ptr<rclcpp::Subscription<CANraw>> can_sub;
uint8_t rad_id, command_id;
bool ack,ready;

std::map<std::string, uint8_t> get_cmd = {
  {"GET_TARGET_ANGLE", CAN_GET_TARGET_ANGLE},
  {"GET_P_VALUE", CAN_GET_P_VALUE},
  {"GET_I_VALUE", CAN_GET_I_VALUE},
  {"GET_D_VALUE", CAN_GET_D_VALUE},
};
std::map<std::string, uint8_t> set_cmd = {
  {"SET_TARGET_ANGLE", CAN_SET_TARGET_ANGLE},
  {"SET_P_VALUE", CAN_SET_P_VALUE},
  {"SET_I_VALUE", CAN_SET_I_VALUE},
  {"SET_D_VALUE", CAN_SET_D_VALUE},
  {"ASSIGN_DEVICE_ID", CAN_ASSIGN_DEVICE_ID},
  {"SET_ODOM_INTERVAL", CAN_SET_ODOM_INTERVAL},
  {"SET_HEALTH_INTERVAL", CAN_SET_HEALTH_INTERVAL}
};
std::map<std::string, uint8_t> other_cmd = {
  {"SAVE_TO_EEPROM", CAN_SAVE_TO_EEPROM}
};


void response_callback(const CANraw& msg)
{
  if (ready && ((msg.address & 0xff) == rad_id) && (((msg.address >> 8) & 0xff) == command_id))
  {
    uint8_t i = 0;
    RCLCPP_INFO(can_config->get_logger(), "Message received!");
    if (command_id == CAN_GET_TARGET_ANGLE || command_id == CAN_GET_P_VALUE || 
        command_id == CAN_GET_I_VALUE || command_id == CAN_GET_D_VALUE)
    {
      RCLCPP_INFO(can_config->get_logger(), "Value: %f", __buffer_get_float64((uint8_t*)&(msg.data[0]),&i));
    }
    else
    {
      RCLCPP_INFO(can_config->get_logger(), "Value: %d", msg.data[0]);
    }
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
  can_sub = can_config->create_subscription<CANraw>("/can/config/rad_can_in", 10, response_callback);
  CANraw can_out_msg;
  RAD rad{&can_out_msg};
  rclcpp::WallRate loop_rate(500ms);

  std::thread spin_thread([](){rclcpp::spin(can_config);});

  std::string in;

  while(true)
  {
    ack = false;
    std::cout << "Enter RAD ID (q to exit) => ";
    std::getline (std::cin,in);
    if (in == "q")
      break;
    rad_id = std::stoi(in);
    rad.set_can_id(rad_id);

    do {
      std::cout << "Enter Command ('?' to list options) => ";
      std::getline (std::cin,in);
      if (in == "?")
      {
        for (const auto&[command_name, _] : get_cmd)
          std::cout << command_name << std::endl;
        for (const auto&[command_name, _] : set_cmd)
          std::cout << command_name << std::endl;
        for (const auto&[command_name, _] : other_cmd)
          std::cout << command_name << std::endl;
      }
    }
    while (in == "?");

    command_id = (get_cmd.count(in) == 1) ? get_cmd.at(in) : 
                 (set_cmd.count(in) == 1 ? set_cmd.at(in) : other_cmd.at(in));

    // SET TYPE COMMAND
    if (set_cmd.count(in) == 1)
    {
      std::string val_in;
      std::cout << "Enter value => ";
      std::getline (std::cin,val_in);

      switch(command_id)
      {
        case CAN_ASSIGN_DEVICE_ID:
          rad.set_can_id((uint8_t)std::stoi(val_in));
          break;
        case CAN_SET_TARGET_ANGLE:
          rad.set_target_angle(std::stod(val_in));
          break;
        case CAN_SET_P_VALUE:
          rad.set_p_value(std::stof(val_in));
          break;
        case CAN_SET_I_VALUE:
          rad.set_i_value(std::stof(val_in));
          break;
        case CAN_SET_D_VALUE:
          rad.set_d_value(std::stof(val_in));
          break;
        case CAN_SET_HEALTH_INTERVAL:
          rad.set_health_interval((uint32_t)std::stoi(val_in));
          break;
        case CAN_SET_ODOM_INTERVAL:
          rad.set_odom_interval((uint32_t)std::stoi(val_in));
          break;
      }
    }
    else if (get_cmd.count(in) == 1) // GET TYPE COMMAND
    {
      switch(command_id)
      {
        case CAN_GET_TARGET_ANGLE:
          rad.get_target_angle();
          break;
        case CAN_GET_P_VALUE:
          rad.get_p_value();
          break;
        case CAN_GET_I_VALUE:
          rad.get_i_value();
          break;
        case CAN_GET_D_VALUE:
          rad.get_d_value();
          break;
      }
    }
    else
    {
      switch (command_id)
      {
        case CAN_SAVE_TO_EEPROM:
          rad.save_to_eeprom();
          break;
      }
    }

    RCLCPP_INFO(can_config->get_logger(), "RAD_ID: %d, COMMAND_ID: %d", rad_id, command_id);
    RCLCPP_INFO(can_config->get_logger(), "SENT CAN FRAME 0x%x", can_out_msg.address);
    can_pub->publish(can_out_msg);

    // Wait for msg receive
    if (get_cmd.count(in) == 1) 
    {
      ready = true;
      int i = 0;
      while (!ack && i++ < 10){
        can_pub->publish(can_out_msg);
        loop_rate.sleep();
      }
      // No reply from RAD
      if (!ack) RCLCPP_ERROR(can_config->get_logger(), "FAILED TO RECEIVE MESSAGE");
      ready = false;
    }
      
  }
  
  spin_thread.~thread();
  rclcpp::shutdown();
  
  return 0;
}