#include "rad_control/rover_steer_test.hpp"

SteerTest::SteerTest(std::string name) : Node(name)
{
    can_pub_ = this->create_publisher<CANraw>("/can/can_out", 10);
    sub_ = this->create_subscription<Int64>(
        "/test/rad_pos", 10, std::bind(&SteerTest::_callback, this, _1)
    );
    timer_ = this->create_wall_timer(
      200ms, std::bind(&SteerTest::_timer_callback, this));
    can_msg.address = 0x55;
    can_msg.data = {0,0,0,0,0,0,0,0};
}

void SteerTest::_callback(const Int64& msg)
{
    int16_t speed = (int16_t)(msg.data);
    if (speed > 0)
    {
        can_msg.address = 0x54;
        can_msg.data[7] = (speed & 0xFF);
    }
    else
    {
        can_msg.address = 0x55;
        can_msg.data[7] = -1*(speed & 0xFF);
    }
}

void SteerTest::_timer_callback()
{
    can_pub_->publish(can_msg);
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
