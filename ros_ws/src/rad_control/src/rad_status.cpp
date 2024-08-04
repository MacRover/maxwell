#include "rad_control/rad_status.hpp"

RAD_Status::RAD_Status() : Node("rad_status_node")
{
  this->declare_parameter("can_topic_1", "/can/rad_can_in_1");
  this->declare_parameter("can_topic_2", "/can/rad_can_in_2");
  this->declare_parameter("can_topic_3", "/can/rad_can_in_3");
  this->declare_parameter("status_rate", 10);

  rate = this->get_parameter("status_rate").as_int();
  topic_1 = this->get_parameter("can_topic_1").as_string();
  topic_2 = this->get_parameter("can_topic_2").as_string();
  topic_3 = this->get_parameter("can_topic_3").as_string();

  status_pub_1 = this->create_publisher<RadStatus>("/front_right/rad_status", 10);
  status_pub_2 = this->create_publisher<RadStatus>("/front_left/rad_status", 10);
  status_pub_3 = this->create_publisher<RadStatus>("/rear_right/rad_status", 10);
  status_pub_4 = this->create_publisher<RadStatus>("/rear_left/rad_status", 10);
  can_sub_1 = this->create_subscription<CANraw>(
    topic_1, 15, std::bind(&RAD_Status::_status_callback_1, this, _1)
  );
  can_sub_2 = this->create_subscription<CANraw>(
    topic_2, 15, std::bind(&RAD_Status::_status_callback_2, this, _1)
  );
  can_sub_3 = this->create_subscription<CANraw>(
    topic_3, 15, std::bind(&RAD_Status::_status_callback_3, this, _1)
  );
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / rate), 
    std::bind(&RAD_Status::_timer_callback, this));
}

void RAD_Status::process_can_msg(const CANraw& msg,
  uint8_t (*decode_can_msg)(const CANraw* can_msg, RadStatus* status))
{
  uint8_t id = (uint8_t)(msg.address & 0xff);

  switch(id)
  {
    case RAD__DRIVE__FRONT_RIGHT:
      (*decode_can_msg)(&msg, &status_1);
      break;
    case RAD__DRIVE__FRONT_LEFT:
      (*decode_can_msg)(&msg, &status_2);
      break;
    case RAD__DRIVE__BACK_RIGHT:
      (*decode_can_msg)(&msg, &status_3);
      break;
    case RAD__DRIVE__BACK_LEFT:
      (*decode_can_msg)(&msg, &status_4);
      break;
  }
}

void RAD_Status::_status_callback_1(const CANraw& msg)
{
  this->process_can_msg(msg, decode_can_status_1);
}

void RAD_Status::_status_callback_2(const CANraw& msg)
{
  this->process_can_msg(msg, decode_can_status_2);
}

void RAD_Status::_status_callback_3(const CANraw& msg)
{
  this->process_can_msg(msg, decode_can_status_3);
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
