#include "viper_control/viper_status.hpp"

VIPER_Status::VIPER_Status() : Node("viper_status_node")
{
  this->declare_parameter("can_topic", "/can/status/viper_can_in");
  this->declare_parameter("status_rate", 10);

  rate = this->get_parameter("status_rate").as_int();
  topic = this->get_parameter("can_topic").as_string();

  status_pub_card_0 = this->create_publisher<ViperCardStatus>("/viper_status/card_0", 10);
  status_pub_card_1 = this->create_publisher<ViperCardStatus>("/viper_status/card_1", 10);
  status_pub_card_2 = this->create_publisher<ViperCardStatus>("/viper_status/card_2", 10);
  status_pub_card_3 = this->create_publisher<ViperCardStatus>("/viper_status/card_3", 10);
  status_pub = this->create_publisher<ViperStatus>("/viper_status/health", 10);
  
  can_sub_ = this->create_subscription<CANraw>(
    topic, 10, std::bind(&VIPER_Status::_status_callback, this, _1)
  );
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / rate), 
    std::bind(&VIPER_Status::_timer_callback, this));
}

void VIPER_Status::_status_callback(const CANraw& msg)
{
  uint8_t id = (uint8_t)(msg.address & 0xff);
  if (id == VIPER__CARD__0)
  {
    decode_can_msg(&msg, &card_0_status, &status);
  }
  else if (id == VIPER__CARD__1)
  {
     decode_can_msg(&msg, &card_1_status, &status);
  }
  else if (id == VIPER__CARD__2)
  {
     decode_can_msg(&msg, &card_2_status, &status);
  }
  else if (id == VIPER__CARD__3)
  {
     decode_can_msg(&msg, &card_3_status, &status);
  }
}

void VIPER_Status::_timer_callback(void)
{
  card_0_status.header.stamp = this->now();
  status_pub_card_0->publish(card_0_status);

  card_1_status.header.stamp = this->now();
  status_pub_card_1->publish(card_1_status);

  card_2_status.header.stamp = this->now();
  status_pub_card_2->publish(card_2_status);

  card_3_status.header.stamp = this->now();
  status_pub_card_3->publish(card_3_status);

  status.header.stamp = this->now();
  status_pub->publish(status);

  
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VIPER_Status>());
  rclcpp::shutdown();
  
  return 0;
}
