#include "rad_control/rad_status.hpp"

RAD_Status::RAD_Status() : Node("rad_status_node")
{
  this->declare_parameter("can_topic", "/can/rad_can_in");
  this->declare_parameter("status_rate", 10);

  rate = this->get_parameter("status_rate").as_int();
  topic = this->get_parameter("can_topic").as_string();

  status_pub_1 = this->create_publisher<RadStatus>("/front_right/rad_status", 10);
  status_pub_2 = this->create_publisher<RadStatus>("/front_left/rad_status", 10);
  status_pub_3 = this->create_publisher<RadStatus>("/rear_right/rad_status", 10);
  status_pub_4 = this->create_publisher<RadStatus>("/rear_left/rad_status", 10);
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
  if (id == RAD__DRIVE__FRONT_RIGHT)
  {
    decode_can_msg(&msg, &status_1);
  }
  else if (id == RAD__DRIVE__FRONT_LEFT)
  {
     decode_can_msg(&msg, &status_2);
  }
  else if (id == RAD__DRIVE__BACK_RIGHT)
  {
     decode_can_msg(&msg, &status_3);
  }
  else if (id == RAD__DRIVE__BACK_LEFT)
  {
     decode_can_msg(&msg, &status_4);
  }
}

void RAD_Status::_timer_callback(void)
{
  status_1.header.stamp = this->now();
  status_pub_1->publish(status_1);

  status_2.header.stamp = this->now();
  status_pub_2->publish(status_2);

  status_3.header.stamp = this->now();
  status_pub_3->publish(status_3);

  status_4.header.stamp = this->now();
  status_pub_4->publish(status_4);
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
