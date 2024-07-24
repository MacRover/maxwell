#include "rad_control/rad_status.hpp"

RAD_Status::RAD_Status() : Node("rad_status_node")
{
  this->declare_parameter("motor_id", 1);
  this->declare_parameter("namespace", "front_right");
  this->declare_parameter("can_topic", "/can/rad_can_in");
  this->declare_parameter("status_rate", 10);

  can_id = this->get_parameter("motor_id").as_int();
  rate = this->get_parameter("status_rate").as_int();
  ns = this->get_parameter("namespace").as_string();
  topic = this->get_parameter("can_topic").as_string();

  status_pub = this->create_publisher<RadStatus>("/" + ns + "/rad_status", 10);
  can_sub_ = this->create_subscription<CANraw>(
    topic, 10, std::bind(&RAD_Status::_status_callback, this, _1)
  );
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / rate), 
    std::bind(&RAD_Status::_timer_callback, this));
}

void RAD_Status::_status_callback(const CANraw& msg)
{
  if ( (uint8_t)(msg.address & 0xff) == can_id)
  {
    decode_can_msg(&msg, &status);
  }
}

void RAD_Status::_timer_callback(void)
{
  status_pub->publish(status);
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RAD_Status>());
  rclcpp::shutdown();
  
  return 0;
}
