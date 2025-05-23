#pragma once
#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/ca_nraw.hpp"
#include "custom_interfaces/msg/ca_nstamped.hpp"
#include "custom_interfaces/msg/swerve_modules_list.hpp"
#include "rad.hpp"

using namespace custom_interfaces::msg;

class RAD_Drive_Controller : public rclcpp::Node
{
public:
    RAD_Drive_Controller();
private:
    void _callback(const SwerveModulesList& msg);
    void _publish_to_can();

    std::shared_ptr<rclcpp::Publisher<CANstamped> > can_pub_;
    std::shared_ptr<rclcpp::Subscription<SwerveModulesList> > sub_;

    CANraw can1_raw, can2_raw, can3_raw, can4_raw;

    RAD rad_fl_drive, rad_fr_drive, rad_bl_drive, rad_br_drive;
    
    double delay_sec;
};