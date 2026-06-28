#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class RoverStateManager : public rclcpp::Node {
public:
  RoverStateManager() : Node("rover_state_manager") {
    // 1. Create the Publisher targeting the Teensy's LED topic
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("/obc/rover_led_state", 10);

    // 2. Create a timer to cycle states for testing (fires every 2 seconds)
    timer_ = this->create_wall_timer(
      2000ms, std::bind(&RoverStateManager::test_state_callback, this));

    RCLCPP_INFO(this->get_logger(), "Rover State Manager initialized. Broadcasting to Teensy...");
  }

private:
  void test_state_callback() {
    auto message = std_msgs::msg::Int32();
    
    // Cycle through states: 1 (Auto/Red), 2 (Teleop/Blue), 3 (Arrived/Green)
    static int current_state = 1;
    message.data = current_state;
    
    RCLCPP_INFO(this->get_logger(), "Publishing LED State: %d", message.data);
    publisher_->publish(message);

    // Increment state for the next timer loop
    current_state++;
    if (current_state > 3) {
      current_state = 1;
    }
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoverStateManager>());
  rclcpp::shutdown();
  return 0;
}
