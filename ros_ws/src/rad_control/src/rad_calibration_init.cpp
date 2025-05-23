#include "rad_control/rad_calibration_init.hpp"
#include <chrono>
#include <thread>

using std::placeholders::_1;

RAD_Init::RAD_Init() : Node("rad_init")
{
   this->declare_parameter<std::vector<int64_t>>("rad_ids", {RAD__DRIVE__FRONT_LEFT, RAD__DRIVE__FRONT_RIGHT});
   this->declare_parameter<std::vector<std::string>>("rad_status", {"front_left/rad_status","front_right/rad_status"});
   this->declare_parameter<std::string>("can_topic", "/can/can_out");
   rad_ids = this->get_parameter("rad_ids").as_integer_array();
   rad_status = this->get_parameter("rad_status").as_string_array();

  if (rad_ids.size() != rad_status.size())
  {
    RCLCPP_ERROR(this->get_logger(), "Unequal mappings between RAD IDs and status topics");
    throw rclcpp::exceptions::InvalidParametersException("Invalid Parameters");
  }

  this->num_of_rads = rad_status.size();
  sub.resize(num_of_rads);

  for (size_t i = 0; i < num_of_rads; i++)
  {
    sub[i] = this->create_subscription<RadStatus>(
      rad_status[i], 10, [this, i](const RadStatus& msg){ this->_callback(msg, i); }
    );
    ls.push_back(false);
  }
}

bool RAD_Init::finished()
{
    return std::all_of(this->ls.begin(), this->ls.end(), [](bool v){ return v; });
}

void RAD_Init::_callback(const RadStatus& msg, int id)
{
    this->ls[id] = msg.ls_state_1;
}

std::shared_ptr<rclcpp::Publisher<CANraw>> can_pub;
std::shared_ptr<RAD_Init> init_node;

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  try {
    rclcpp::init(argc, argv);
    init_node = std::make_shared<RAD_Init>();
  }
  catch (...)
  {
    rclcpp::shutdown();
    exit(1);
  }

  std::thread spin_thread([](){rclcpp::spin(init_node);});

  std::vector<CANraw> cans(init_node->num_of_rads);
  std::vector<RAD> rads;

  for (size_t i = 0; i < init_node->num_of_rads; i++)
  {
    cans[i] = CANraw();
    rads.push_back(RAD(&cans[i], init_node->rad_ids[i]));
  }

  std::string can_topic = init_node->get_parameter("can_topic").as_string();
  can_pub = init_node->create_publisher<CANraw>(can_topic, 10);
  rclcpp::Rate rate{std::chrono::milliseconds(1000)};

  RCLCPP_INFO(init_node->get_logger(), "Zeroing RAD motors");
  
  // Constantly rotate motors until it hits limit switch
  // Quit once all motors have been calibrated
  while (!init_node->finished() && rclcpp::ok())
  {
    for (size_t i = 0; i < init_node->num_of_rads; i++)
    {
      rads[i].calibrate_zero_pos();
      can_pub->publish(cans[i]);
    }
    rate.sleep();
  }

  // kill node and spin thread
  spin_thread.~thread();
  rclcpp::shutdown();

  return 0;
}