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
  {"GET_STEPPER_SPEED", CAN_GET_STEPPER_SPEED},
  {"GET_P_VALUE", CAN_GET_P_VALUE},
  {"GET_I_VALUE", CAN_GET_I_VALUE},
  {"GET_D_VALUE", CAN_GET_D_VALUE},
  {"GET_RAD_TYPE", CAN_GET_RAD_TYPE},
  {"GET_HOME_POSITION", CAN_GET_HOME_POSITION},
  {"GET_ODOM_INTERVAL", CAN_GET_ODOM_INTERVAL},
  {"GET_HEALTH_INTERVAL", CAN_GET_HEALTH_INTERVAL},
  {"GET_DRVCONF_TST", CAN_GET_DRVCONF_TST},
  {"GET_DRVCONF_SLP", CAN_GET_DRVCONF_SLP},
  {"GET_DRVCONF_S2G", CAN_GET_DRVCONF_S2G},
  {"GET_DRVCONF_TS2G", CAN_GET_DRVCONF_TS2G},
  {"GET_DRVCONF_SDOFF", CAN_GET_DRVCONF_SDOFF},
  {"GET_DRVCONF_VSENSE", CAN_GET_DRVCONF_VSENSE},
  {"GET_DRVCONF_RDSEL", CAN_GET_DRVCONF_RDSEL},
  {"GET_DRVCONF_OTSENS", CAN_GET_DRVCONF_OTSENS},
  {"GET_DRVCONF_SHRTSENS", CAN_GET_DRVCONF_SHRTSENS},
  {"GET_DRVCONF_EN_PFD", CAN_GET_DRVCONF_EN_PFD},
  {"GET_DRVCONF_EN_S2VS", CAN_GET_DRVCONF_EN_S2VS},
  {"GET_SGCSCONF_SFILT", CAN_GET_SGCSCONF_SFILT},
  {"GET_SGCSCONF_SGT", CAN_GET_SGCSCONF_SGT},
  {"GET_SGCSCONF_CS", CAN_GET_SGCSCONF_CS},
  {"GET_SMARTEN_SEIMIN", CAN_GET_SMARTEN_SEIMIN},
  {"GET_SMARTEN_SEDN", CAN_GET_SMARTEN_SEDN},
  {"GET_SMARTEN_SEMAX", CAN_GET_SMARTEN_SEMAX},
  {"GET_SMARTEN_SEUP", CAN_GET_SMARTEN_SEUP},
  {"GET_SMARTEN_SEMIN", CAN_GET_SMARTEN_SEMIN},
  {"GET_CHOPCONF_TBL", CAN_GET_CHOPCONF_TBL},
  {"GET_CHOPCONF_CHM", CAN_GET_CHOPCONF_CHM},
  {"GET_CHOPCONF_RNDTF", CAN_GET_CHOPCONF_RNDTF},
  {"GET_CHOPCONF_HDEC", CAN_GET_CHOPCONF_HDEC},
  {"GET_CHOPCONF_HEND", CAN_GET_CHOPCONF_HEND},
  {"GET_CHOPCONF_HSTRT", CAN_GET_CHOPCONF_HSTRT},
  {"GET_CHOPCONF_TOFF", CAN_GET_CHOPCONF_TOFF},
  {"GET_DRVCTRL_INTPOL", CAN_GET_DRVCTRL_INTPOL},
  {"GET_DRVCTRL_DEDGE", CAN_GET_DRVCTRL_DEDGE},
  {"GET_DRVCTRL_MRES", CAN_GET_DRVCTRL_MRES},
  {"GET_PID_MIN_OUTPUT", CAN_GET_PID_MIN_OUTPUT},
  {"GET_PID_MAX_OUTPUT", CAN_GET_PID_MAX_OUTPUT},
  {"GET_HOME_OFFSET", CAN_GET_HOME_OFFSET}
};
std::map<std::string, uint8_t> set_cmd = {
  {"SET_TARGET_ANGLE", CAN_SET_TARGET_ANGLE},
  {"SET_STEPPER_SPEED", CAN_SET_STEPPER_SPEED},
  {"SET_P_VALUE", CAN_SET_P_VALUE},
  {"SET_I_VALUE", CAN_SET_I_VALUE},
  {"SET_D_VALUE", CAN_SET_D_VALUE},
  {"SET_RAD_TYPE", CAN_SET_RAD_TYPE},
  {"SET_HOME_POSITION", CAN_SET_HOME_POSITION},
  {"SET_ODOM_INTERVAL", CAN_SET_ODOM_INTERVAL},
  {"SET_HEALTH_INTERVAL", CAN_SET_HEALTH_INTERVAL},
  {"SET_DRVCONF_TST", CAN_SET_DRVCONF_TST},
  {"SET_DRVCONF_SLP", CAN_SET_DRVCONF_SLP},
  {"SET_DRVCONF_S2G", CAN_SET_DRVCONF_S2G},
  {"SET_DRVCONF_TS2G", CAN_SET_DRVCONF_TS2G},
  {"SET_DRVCONF_SDOFF", CAN_SET_DRVCONF_SDOFF},
  {"SET_DRVCONF_VSENSE", CAN_SET_DRVCONF_VSENSE},
  {"SET_DRVCONF_RDSEL", CAN_SET_DRVCONF_RDSEL},
  {"SET_DRVCONF_OTSENS", CAN_SET_DRVCONF_OTSENS},
  {"SET_DRVCONF_SHRTSENS", CAN_SET_DRVCONF_SHRTSENS},
  {"SET_DRVCONF_EN_PFD", CAN_SET_DRVCONF_EN_PFD},
  {"SET_DRVCONF_EN_S2VS", CAN_SET_DRVCONF_EN_S2VS},
  {"SET_SGCSCONF_SFILT", CAN_SET_SGCSCONF_SFILT},
  {"SET_SGCSCONF_SGT", CAN_SET_SGCSCONF_SGT},
  {"SET_SGCSCONF_CS", CAN_SET_SGCSCONF_CS},
  {"SET_SMARTEN_SEIMIN", CAN_SET_SMARTEN_SEIMIN},
  {"SET_SMARTEN_SEDN", CAN_SET_SMARTEN_SEDN},
  {"SET_SMARTEN_SEMAX", CAN_SET_SMARTEN_SEMAX},
  {"SET_SMARTEN_SEUP", CAN_SET_SMARTEN_SEUP},
  {"SET_SMARTEN_SEMIN", CAN_SET_SMARTEN_SEMIN},
  {"SET_CHOPCONF_TBL", CAN_SET_CHOPCONF_TBL},
  {"SET_CHOPCONF_CHM", CAN_SET_CHOPCONF_CHM},
  {"SET_CHOPCONF_RNDTF", CAN_SET_CHOPCONF_RNDTF},
  {"SET_CHOPCONF_HDEC", CAN_SET_CHOPCONF_HDEC},
  {"SET_CHOPCONF_HEND", CAN_SET_CHOPCONF_HEND},
  {"SET_CHOPCONF_HSTRT", CAN_SET_CHOPCONF_HSTRT},
  {"SET_CHOPCONF_TOFF", CAN_SET_CHOPCONF_TOFF},
  {"SET_DRVCTRL_INTPOL", CAN_SET_DRVCTRL_INTPOL},
  {"SET_DRVCTRL_DEDGE", CAN_SET_DRVCTRL_DEDGE},
  {"SET_DRVCTRL_MRES", CAN_SET_DRVCTRL_MRES},
  {"SET_PID_MIN_OUTPUT", CAN_SET_PID_MIN_OUTPUT},
  {"SET_PID_MAX_OUTPUT", CAN_SET_PID_MAX_OUTPUT},
  {"ASSIGN_DEVICE_ID", CAN_ASSIGN_DEVICE_ID},
  {"PULSE_STEPPER", CAN_PULSE_STEPPER},
  {"SET_MAX_POINT", CAN_SET_MAX_POINT}
};
std::map<std::string, uint8_t> other_cmd = {
  {"SET_HOME_OFFSET", CAN_SET_HOME_OFFSET},
  {"SAVE_TO_EEPROM", CAN_SAVE_TO_EEPROM},
  {"RELOAD_FROM_EEPROM", CAN_RELOAD_FROM_EEPROM},
  {"CALIBRATE", CAN_CALIBRATE_POS},
  {"CANCEL_CALIBRATE", CAN_CANCEL_CALIBRATE_POS},
  {"SET_ZERO_POINT", CAN_SET_ZERO_POINT},
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
      RCLCPP_INFO(can_config->get_logger(), "Value: %d", msg.data.back());
    }
    ack = true;
  }
}

bool print_autocompleted_commands(std::map<std::string, uint8_t> map, std::string in)
{
  if (in.length() == 0)
  {
    return false;
  }

  bool ret = false;
  for (const auto&[command_name, _] : map)
  {
    if (command_name.compare(0, in.length(), in) == 0 && command_name.length() != in.length())
    {
      std::cout << command_name << "  ";
      ret = true;
    }
  }
  return ret;
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
    // Removing whitespace
    in.erase(std::remove_if(in.begin(), in.end(), isspace), in.end());
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
      // Removing whitespace and capitalize
      in.erase(std::remove_if(in.begin(), in.end(), isspace), in.end());
      std::transform(in.begin(), in.end(), in.begin(), toupper);
      if (in == "?")
      {
        // List out all RAD commands
        std::cout << " ========= SET COMMANDS =========" << std::endl;
        for (const auto&[command_name, _] : set_cmd)
          std::cout << command_name << std::endl;
        std::cout << " ========= GET COMMANDS =========" << std::endl;
        for (const auto&[command_name, _] : get_cmd)
          std::cout << command_name << std::endl;
        std::cout << " ========= CONFIG COMMANDS =========" << std::endl;
        for (const auto&[command_name, _] : other_cmd)
          std::cout << command_name << std::endl;
      }
      else 
      {
        bool valid_input = false;
        valid_input = valid_input || print_autocompleted_commands(get_cmd, in);
        valid_input = valid_input || print_autocompleted_commands(set_cmd, in);
        valid_input = valid_input || print_autocompleted_commands(other_cmd, in);
        if (!valid_input)
          break;
        std::cout << std::endl;
      }
    }
    while (!get_cmd.count(in) && !set_cmd.count(in) && !other_cmd.count(in));

    // Input check here to catch valid_input if it is false
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
      // Removing whitespace
      in.erase(std::remove_if(in.begin(), in.end(), isspace), in.end());

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
        case CAN_SET_STEPPER_SPEED:
          rad.set_stepper_speed(std::stof(val_in));
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
        case CAN_SET_RAD_TYPE:
          rad.set_rad_type((uint8_t)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_HOME_POSITION:
          rad.set_home_position((uint32_t)std::stoul(val_in, 0, base));
          break;
        case CAN_SET_ODOM_INTERVAL:
          rad.set_odom_interval((uint32_t)std::stoul(val_in, 0, base));
          break;
        case CAN_SET_HEALTH_INTERVAL:
          rad.set_health_interval((uint32_t)std::stoul(val_in, 0, base));
          break;
        case CAN_SET_DRVCONF_TST:
          rad.set_drvconf_tst((bool)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_DRVCONF_SLP:
          rad.set_drvconf_slp((uint8_t)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_DRVCONF_S2G:
          rad.set_drvconf_s2g((bool)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_DRVCONF_TS2G:
          rad.set_drvconf_ts2g((uint8_t)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_DRVCONF_SDOFF:
          rad.set_drvconf_sdoff((bool)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_DRVCONF_VSENSE:
          rad.set_drvconf_vsense((bool)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_DRVCONF_RDSEL:
          rad.set_drvconf_rdsel((uint8_t)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_DRVCONF_OTSENS:
          rad.set_drvconf_otsens((bool)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_DRVCONF_SHRTSENS:
          rad.set_drvconf_shrtsens((bool)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_DRVCONF_EN_PFD:
          rad.set_drvconf_en_pfd((bool)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_DRVCONF_EN_S2VS:
          rad.set_drvconf_en_s2vs((bool)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_SGCSCONF_SFILT:
          rad.set_sgcsconf_sfilt((bool)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_SGCSCONF_SGT:
          rad.set_sgcsconf_sgt((int8_t)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_SGCSCONF_CS:
          rad.set_sgcsconf_cs((uint8_t)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_SMARTEN_SEIMIN:
          rad.set_smarten_seimin((bool)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_SMARTEN_SEDN:
          rad.set_smarten_sedn((uint8_t)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_SMARTEN_SEMAX:
          rad.set_smarten_semax((uint8_t)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_SMARTEN_SEUP:
          rad.set_smarten_seup((uint8_t)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_SMARTEN_SEMIN:
          rad.set_smarten_semin((uint8_t)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_CHOPCONF_TBL:
          rad.set_chopconf_tbl((uint8_t)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_CHOPCONF_CHM:
          rad.set_chopconf_chm((bool)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_CHOPCONF_RNDTF:
          rad.set_chopconf_rndtf((bool)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_CHOPCONF_HDEC:
          rad.set_chopconf_hdec((uint8_t)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_CHOPCONF_HEND:
          rad.set_chopconf_hend((uint8_t)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_CHOPCONF_HSTRT:
          rad.set_chopconf_hstrt((uint8_t)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_CHOPCONF_TOFF:
          rad.set_chopconf_toff((uint8_t)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_DRVCTRL_INTPOL:
          rad.set_drvctrl_intpol((bool)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_DRVCTRL_DEDGE:
          rad.set_drvctrl_dedge((bool)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_DRVCTRL_MRES:
          rad.set_drvctrl_mres((uint8_t)std::stoi(val_in, 0, base));
          break;
        case CAN_SET_PID_MIN_OUTPUT:
          rad.set_min_output((uint16_t)std::stoul(val_in, 0, base));
          break;
        case CAN_SET_PID_MAX_OUTPUT:
          rad.set_max_output((uint16_t)std::stoul(val_in, 0, base));
          break;
        case CAN_PULSE_STEPPER:
          rad.pulse_stepper(std::stof(val_in));
          break;
        case CAN_SET_MAX_POINT:
          rad.set_max_point((uint8_t)std::stoi(val_in, 0, base));
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
        case CAN_GET_STEPPER_SPEED:
          rad.get_stepper_speed();
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
        case CAN_GET_RAD_TYPE:
          rad.get_rad_type();
          break;
        case CAN_GET_HOME_POSITION:
          rad.get_home_position();
          break;
        case CAN_GET_ODOM_INTERVAL:
          rad.get_odom_interval();
          break;
        case CAN_GET_HEALTH_INTERVAL:
          rad.get_health_interval();
          break;
        case CAN_GET_DRVCONF_TST:
          rad.get_drvconf_tst();
          break;
        case CAN_GET_DRVCONF_SLP:
          rad.get_drvconf_slp();
          break;
        case CAN_GET_DRVCONF_S2G:
          rad.get_drvconf_s2g();
          break;
        case CAN_GET_DRVCONF_TS2G:
          rad.get_drvconf_ts2g();
          break;
        case CAN_GET_DRVCONF_SDOFF:
          rad.get_drvconf_sdoff();
          break;
        case CAN_GET_DRVCONF_VSENSE:
          rad.get_drvconf_vsense();
          break;
        case CAN_GET_DRVCONF_RDSEL:
          rad.get_drvconf_rdsel();
          break;
        case CAN_GET_DRVCONF_OTSENS:
          rad.get_drvconf_otsens();
          break;
        case CAN_GET_DRVCONF_SHRTSENS:
          rad.get_drvconf_shrtsens();
          break;
        case CAN_GET_DRVCONF_EN_PFD:
          rad.get_drvconf_en_pfd();
          break;
        case CAN_GET_DRVCONF_EN_S2VS:
          rad.get_drvconf_en_s2vs();
          break;
        case CAN_GET_SGCSCONF_SFILT:
          rad.get_sgcsconf_sfilt();
          break;
        case CAN_GET_SGCSCONF_SGT:
          rad.get_sgcsconf_sgt();
          break;
        case CAN_GET_SGCSCONF_CS:
          rad.get_sgcsconf_cs();
          break;
        case CAN_GET_SMARTEN_SEIMIN:
          rad.get_smarten_seimin();
          break;
        case CAN_GET_SMARTEN_SEDN:
          rad.get_smarten_sedn();
          break;
        case CAN_GET_SMARTEN_SEMAX:
          rad.get_smarten_semax();
          break;
        case CAN_GET_SMARTEN_SEUP:
          rad.get_smarten_seup();
          break;
        case CAN_GET_SMARTEN_SEMIN:
          rad.get_smarten_semin();
          break;
        case CAN_GET_CHOPCONF_TBL:
          rad.get_chopconf_tbl();
          break;
        case CAN_GET_CHOPCONF_CHM:
          rad.get_chopconf_chm();
          break;
        case CAN_GET_CHOPCONF_RNDTF:
          rad.get_chopconf_rndtf();
          break;
        case CAN_GET_CHOPCONF_HDEC:
          rad.get_chopconf_hdec();
          break;
        case CAN_GET_CHOPCONF_HEND:
          rad.get_chopconf_hend();
          break;
        case CAN_GET_CHOPCONF_HSTRT:
          rad.get_chopconf_hstrt();
          break;
        case CAN_GET_CHOPCONF_TOFF:
          rad.get_chopconf_toff();
          break;
        case CAN_GET_DRVCTRL_INTPOL:
          rad.get_drvctrl_intpol();
          break;
        case CAN_GET_DRVCTRL_DEDGE:
          rad.get_drvctrl_dedge();
          break;
        case CAN_GET_DRVCTRL_MRES:
          rad.get_drvctrl_mres();
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
        case CAN_RELOAD_FROM_EEPROM:
          rad.reload_from_eeprom();
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
        case CAN_SET_ZERO_POINT:
          rad.set_zero_point();
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