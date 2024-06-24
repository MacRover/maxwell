#pragma once
#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/ca_nraw.hpp"
#include "custom_interfaces/msg/swerve_modules_list.hpp"
#include "rad.hpp"

using namespace custom_interfaces::msg;

class RAD_Drive_Controller : public rclcpp::Node
{
public:
    RAD_Drive_Controller();
private:
    void _callback(const SwerveModulesList& msg);
    std::shared_ptr<rclcpp::Publisher<CANraw> > can_pub_;
    std::shared_ptr<rclcpp::Subscription<SwerveModulesList> > sub_;

    uint16_t sleep_msec;
};