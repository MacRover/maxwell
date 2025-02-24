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
  {"GET_PID_MIN_OUTPUT", CAN_GET_PID_MIN_OUTPUT},
  {"GET_PID_MAX_OUTPUT", CAN_GET_PID_MAX_OUTPUT},
  {"GET_HOME_OFFSET", CAN_GET_HOME_OFFSET}

};
std::map<std::string, uint8_t> set_cmd = {
  {"SET_TARGET_ANGLE", CAN_SET_TARGET_ANGLE},
  {"SET_P_VALUE", CAN_SET_P_VALUE},
  {"SET_I_VALUE", CAN_SET_I_VALUE},
  {"SET_D_VALUE", CAN_SET_D_VALUE},
  {"ASSIGN_DEVICE_ID", CAN_ASSIGN_DEVICE_ID},
  {"SET_ODOM_INTERVAL", CAN_SET_ODOM_INTERVAL},
  {"SET_HEALTH_INTERVAL", CAN_SET_HEALTH_INTERVAL},
  {"SET_RAD_TYPE", CAN_SET_RAD_TYPE},
  {"SET_DRVCTRL_MRES", CAN_SET_DRVCTRL_MRES},
  {"SET_SGCSCONF_CS", CAN_SET_SGCSCONF_CS},
  {"SET_STEPPER_SPEED", CAN_SET_STEPPER_SPEED},
  {"SET_PID_MIN_OUTPUT", CAN_SET_PID_MIN_OUTPUT},
  {"SET_PID_MAX_OUTPUT", CAN_SET_PID_MAX_OUTPUT},
  {"PULSE_STEPPER", CAN_PULSE_STEPPER}
};
std::map<std::string, uint8_t> other_cmd = {
  {"CALIBRATE", CAN_CALIBRATE_POS},
  {"CANCEL_CALIBRATE", CAN_CANCEL_CALIBRATE_POS},
  {"SAVE_TO_EEPROM", CAN_SAVE_TO_EEPROM},
  {"SET_HOME_OFFSET", CAN_SET_HOME_OFFSET}
};


void response_callback(const CANraw& msg)
{
  if (ready && ((msg.address & 0xff) == rad_id) && (((msg.address >> 8) & 0xff) == command_id))
  {
    uint8_t i = 0;
    RCLCPP_INFO(can_config->get_logger(), "Message received!");
    if (command_id == CAN_GET_TARGET_ANGLE || command_id == CAN_GET_P_VALUE || 
        command_id == CAN_GET_I_VALUE || command_id == CAN_GET_D_VALUE || command_id == CAN_GET_HOME_OFFSET)
    {
      RCLCPP_INFO(can_config->get_logger(), "Value: %f", __buffer_get_float64((uint8_t*)&(msg.data[0]),&i));
    }
    else if (command_id == CAN_GET_PID_MIN_OUTPUT || command_id == CAN_GET_PID_MAX_OUTPUT)
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
    std::cout << "Enter RAD ID (q to exit) (prefix h for hex #) => ";
    std::getline (std::cin,in);
    if (in == "q" || std::cin.fail())
      break;
    
    int base = 10;
    INPUT_CHECK(
    if (in[0] == 'h')
    {
      base = 16;
      in = in.substr(1);
    }
    rad_id = std::stoi(in, 0, base);
    rad.set_can_id(rad_id);
    )

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

    INPUT_CHECK(
    command_id = (get_cmd.count(in) == 1) ? get_cmd.at(in) : 
                 (set_cmd.count(in) == 1 ? set_cmd.at(in) : other_cmd.at(in));
    )

    // SET TYPE COMMAND
    if (set_cmd.count(in) == 1)
    {
      std::string val_in;
      std::cout << "Enter value (prefix h for hex #) => ";
      std::getline (std::cin,val_in);

      base = 10;
      INPUT_CHECK(
      if (val_in[0] == 'h')
      {
        base = 16;
        val_in = val_in.substr(1);
      }
      )

      switch(command_id)
      {
        case CAN_ASSIGN_DEVICE_ID:
          rad.set_can_id((uint8_t)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_TARGET_ANGLE:
          rad.set_target_angle(std::stod(val_in));
          break;
        case CAN_SET_P_VALUE:
          rad.set_p_value(std::stod(val_in));
          break;
        case CAN_SET_I_VALUE:
          rad.set_i_value(std::stod(val_in));
          break;
        case CAN_SET_D_VALUE:
          rad.set_d_value(std::stod(val_in));
          break;
        case CAN_SET_HEALTH_INTERVAL:
          rad.set_health_interval((uint32_t)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_ODOM_INTERVAL:
          rad.set_odom_interval((uint32_t)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_RAD_TYPE:
          rad.set_rad_type((uint8_t)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_DRVCTRL_MRES:
          rad.set_drvctrl_mres((uint8_t)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_SGCSCONF_CS:
          rad.set_sgcsconf_cs((uint8_t)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_STEPPER_SPEED:
          rad.set_stepper_speed((uint32_t)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_PID_MIN_OUTPUT:
          rad.set_min_output((uint8_t)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_PID_MAX_OUTPUT:
          rad.set_max_output((uint16_t)std::stoi(val_in, 0, base));
          break;
        case CAN_PULSE_STEPPER:
          rad.pulse_stepper(std::stof(val_in));
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
        case CAN_GET_PID_MIN_OUTPUT:
          rad.get_min_output();
          break;
        case CAN_GET_PID_MAX_OUTPUT:
          rad.get_max_output();
          break;
        case CAN_GET_HOME_OFFSET:
          rad.get_home_offset();
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
        case CAN_CALIBRATE_POS:
          rad.calibrate_zero_pos();
          break;
        case CAN_CANCEL_CALIBRATE_POS:
          rad.cancel_calibration();
          break;
        case CAN_SET_HOME_OFFSET:
          rad.set_home_offset();
          break;
      }
    }

    RCLCPP_INFO(can_config->get_logger(), "RAD_ID: %d, COMMAND_ID: %d", rad_id, command_id);
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
      // No reply from RAD
      if (!ack) RCLCPP_ERROR(can_config->get_logger(), "FAILED TO RECEIVE MESSAGE");
      ready = false;
    }
      
  }
  
  spin_thread.~thread();
  rclcpp::shutdown();
  
  return 0;
}