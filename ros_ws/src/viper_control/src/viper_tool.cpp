#include <thread>
#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/ca_nraw.hpp"
#include "viper_control/viper.hpp"

using namespace std::chrono_literals;
using namespace custom_interfaces::msg;
using std::placeholders::_1;

std::shared_ptr<rclcpp::Node> can_config;
std::shared_ptr<rclcpp::Publisher<CANraw>> can_pub;
std::shared_ptr<rclcpp::Subscription<CANraw>> can_sub;
uint8_t viper_card_id, command_id;
bool ack,ready;

std::map<std::string, uint8_t> get_cmd = {
  {"GET_CARD_INTERVAL", CAN_GET_CARD_INTERVAL},
  {"GET_HEALTH_INTERVAL", CAN_GET_HEALTH_INTERVAL}

};
std::map<std::string, uint8_t> set_cmd = {
  {"SET_HEALTH_INTERVAL", CAN_SET_HEALTH_INTERVAL},
  {"SET_CARD_INTERVAL", CAN_SET_CARD_INTERVAL},
  {"DISABLE_CARD", CAN_DISABLE_CARD},
  {"ENABLE_CARD", CAN_ENABLE_CARD},
  {"SET_MUX_VALUE", CAN_SET_MUX_VALUE}
};
std::map<std::string, uint8_t> other_cmd = {
  {"SAVE_TO_EEPROM", CAN_SAVE_TO_EEPROM},
  {"DISABLE_ALL_CARDS", CAN_DISABLE_ALL_CARDS},
  {"ENABLE_ALL_CARDS", CAN_ENABLE_ALL_CARDS}
};


void response_callback(const CANraw& msg)
{
  if (ready && ((msg.address & 0xff) == viper_card_id) && (((msg.address >> 8) & 0xff) == command_id))
  {
    //uint8_t i = 0;
    RCLCPP_INFO(can_config->get_logger(), "Message received!");
    // if (command_id == CAN_GET_TARGET_ANGLE || command_id == CAN_GET_P_VALUE || 
    //     command_id == CAN_GET_I_VALUE || command_id == CAN_GET_D_VALUE || command_id == CAN_GET_HOME_OFFSET)
    // {
    //   RCLCPP_INFO(can_config->get_logger(), "Value: %f", __buffer_get_float64((uint8_t*)&(msg.data[0]),&i));
    // }
    if (command_id == CAN_GET_HEALTH_INTERVAL || command_id == CAN_GET_CARD_INTERVAL)
    {
      RCLCPP_INFO(can_config->get_logger(), "Value: %d", ((uint16_t)(msg.data[0]) << 8) | msg.data[1]);
    }
    else
    {
      RCLCPP_INFO(can_config->get_logger(), "Value: %d", msg.data[0]);
    }
    ack = true;
  }
}

#define INPUT_CHECK(c) try {c} catch(...){RCLCPP_ERROR(can_config->get_logger(), "INVALID INPUT"); continue;}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  
  can_config = std::make_shared<rclcpp::Node>("viper_tool_node");
  can_pub = can_config->create_publisher<CANraw>("/can/can_out", 10);
  can_sub = can_config->create_subscription<CANraw>("/can/config/viper_can_in", 10, response_callback);
  CANraw can_out_msg;
  VIPER viper{&can_out_msg, 1};
  rclcpp::WallRate loop_rate(500);

  std::thread spin_thread([](){rclcpp::spin(can_config);});

  std::string in;

  while(true)
  {
    ack = false;

    //THIS IS UNECESSARY!
    // std::cout << "Enter Card ID (q to exit) (prefix h for hex #) => ";
    // std::getline (std::cin,in)
    // if (in == "q" || std::cin.fail())
    //   break;
    
    // int base = 10;
    // INPUT_CHECK(
    // if (in[0] == 'h')
    // {
    //   base = 16;
    //   in = in.substr(1);
    // }
    viper_card_id = 0xFF;
    // //viper_card_id.set_can_id(viper_card_id);
    // )

    do {
      std::cout << "Enter Command ('?' to list options) (q to exit) => ";
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

    if (in == "q" || std::cin.fail())
      break;

    INPUT_CHECK(
    command_id = (get_cmd.count(in) == 1) ? get_cmd.at(in) : 
                 (set_cmd.count(in) == 1 ? set_cmd.at(in) : other_cmd.at(in));
    )

    // SET TYPE COMMAND
    if (set_cmd.count(in) == 1)
    {
      std::string val_in;
      std::cout << "Enter value or Card ID (prefix h for hex #) => ";
      std::getline (std::cin,val_in);

      int base = 10;
      INPUT_CHECK(
      if (val_in[0] == 'h')
      {
        base = 16;
        val_in = val_in.substr(1);
      }
      )

      switch(command_id)
      {
        case CAN_SET_HEALTH_INTERVAL:
          viper.set_health_interval((uint32_t)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_CARD_INTERVAL:
          viper.set_card_interval((uint32_t)std::stoi(val_in, 0, base));
          break;
        case CAN_DISABLE_CARD:
          viper_card_id = (uint8_t)std::stoi(val_in, 0, base);
          viper.disable_card(viper_card_id);
          break;
        case CAN_ENABLE_CARD:
          viper_card_id = (uint8_t)std::stoi(val_in, 0, base);
          viper.enable_card(viper_card_id);
          break;
        case CAN_SET_MUX_VALUE:
          viper.set_mux_value((uint8_t)std::stoi(val_in, 0, base));
          break;
      }
    }
    else if (get_cmd.count(in) == 1) // GET TYPE COMMAND
    {
      switch(command_id)
      {
        case CAN_GET_CARD_INTERVAL:
          viper.get_card_interval();
          break;
        case CAN_GET_HEALTH_INTERVAL:
          viper.get_card_interval();
          break;
        
      }
    }
    else
    {
      switch (command_id)
      {
        case CAN_SAVE_TO_EEPROM:
          viper.save_to_eeprom();
          break;
        case CAN_DISABLE_ALL_CARDS:
          viper.disable_all_cards();
          break;
        case CAN_ENABLE_ALL_CARDS:
          viper.enable_all_cards();
          break;

      }
    }

    RCLCPP_INFO(can_config->get_logger(), "VIPER_CARD_ID: %d, COMMAND_ID: %d", viper_card_id, command_id);
    can_pub->publish(can_out_msg);
    RCLCPP_INFO(can_config->get_logger(), "SENT CAN FRAME 0x%x", can_out_msg.address);

    // Wait for msg receive
    if (get_cmd.count(in) == 1) 
    {
      ready = true;
      int i = 0;
      while (!ack && i++ < 10){
        can_pub->publish(can_out_msg);
        loop_rate.sleep();
      }
      // No reply from VIPER
      if (!ack) RCLCPP_ERROR(can_config->get_logger(), "FAILED TO RECEIVE MESSAGE");
      ready = false;
    }
      
  }
  
  spin_thread.~thread();
  rclcpp::shutdown();
  
  return 0;
}