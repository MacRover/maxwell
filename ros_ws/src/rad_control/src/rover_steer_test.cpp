#include "rad_control/rover_steer_test.hpp"

SteerTest::SteerTest(std::string name) : Node(name)
{
    can_pub_ = this->create_publisher<CANraw>("/can/can_out", 10);
    sub_fr_ = this->create_subscription<Int64>(
        "/test/front_right/rad_speed", 10, std::bind(&SteerTest::_callback_fr, this, _1)
    );
    sub_fl_ = this->create_subscription<Int64>(
        "/test/front_left/rad_speed", 10, std::bind(&SteerTest::_callback_fl, this, _1)
    );
    timer_ = this->create_wall_timer(
      200ms, std::bind(&SteerTest::_timer_callback, this));
    can_msg_1.address = 0x155;
    can_msg_1.data = {0,0,0,0,0,0,0,0};
    can_msg_2.address = 0x255;
    can_msg_2.data = {0,0,0,0,0,0,0,0};
}

void SteerTest::_callback_fr(const Int64& msg)
{
    int16_t speed = (int16_t)(msg.data);
    if (speed > 0)
    {
        can_msg_1.address = 0x154;
        can_msg_1.data[7] = (speed & 0xFF);
    }
    else
    {
        can_msg_1.address = 0x155;
        can_msg_1.data[7] = -1*(speed & 0xFF);
    }
}

void SteerTest::_callback_fl(const Int64& msg)
{
    int16_t speed = (int16_t)(msg.data);
    if (speed > 0)
    {
        can_msg_2.address = 0x254;
        can_msg_2.data[7] = (speed & 0xFF);
    }
    else
    {
        can_msg_2.address = 0x255;
        can_msg_2.data[7] = -1*(speed & 0xFF);
    }
}

void SteerTest::_timer_callback()
{
    can_pub_->publish(can_msg_1);
    can_pub_->publish(can_msg_2);
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SteerTest>("steer_test_node"));
  rclcpp::shutdown();

  return 0;
}
