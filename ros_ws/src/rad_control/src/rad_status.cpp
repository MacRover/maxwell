#include "rad_control/rad_status.hpp"

RAD_Status::RAD_Status() : Node("rad_status_node")
{
  this->declare_parameter("can_topic", "/can/status/rad_can_in");
  this->declare_parameter("status_rate", 10);
  this->declare_parameter<std::vector<int64_t>>("rad_ids", {RAD__DRIVE__FRONT_LEFT, RAD__DRIVE__FRONT_RIGHT});
  this->declare_parameter<std::vector<std::string>>("rad_status", {"/drive/front_left/rad_status", "/drive/front_right/rad_status"});

  rate = this->get_parameter("status_rate").as_int();
  topic = this->get_parameter("can_topic").as_string();
  rad_ids = this->get_parameter("rad_ids").as_integer_array();
  rad_status = this->get_parameter("rad_status").as_string_array();

  if (rad_ids.size() != rad_status.size())
  {
    RCLCPP_ERROR(this->get_logger(),"Unequal mappings between RAD IDs and status topics");
    throw rclcpp::exceptions::InvalidParametersException("Invalid Parameters");
  }

  status_pub.resize(rad_status.size());
  status_topic.resize(rad_status.size());

  for (size_t i = 0; i < status_pub.size(); i++)
  {
    status_pub[i] = this->create_publisher<RadStatus>(rad_status[i], 10);
    status_topic[i] = RadStatus();
  }

  can_sub_ = this->create_subscription<CANraw>(
    topic, 10, std::bind(&RAD_Status::_status_callback, this, _1)
  );
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / rate), 
    std::bind(&RAD_Status::_timer_callback, this));
}

void RAD_Status::_status_callback(const CANraw& msg)
{
  uint8_t id = (uint8_t)(msg.address & 0xff);
  for (size_t i = 0; i < rad_ids.size(); i++)
  {
    if (id == (uint8_t)rad_ids[i])
    {
      decode_can_msg(&msg, &status_topic[i]);
      break;
    }
  }
}

void RAD_Status::_timer_callback(void)
{
  for (size_t i = 0; i < status_pub.size(); i++)
  {
    status_topic[i].header.stamp = this->now();
    status_pub[i]->publish(status_topic[i]);
  }
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  try {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RAD_Status>());
    rclcpp::shutdown();
  }
  catch (...)
  {
    exit(1);
  }
  
  return 0;
}
