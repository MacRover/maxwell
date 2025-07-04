#pragma once
#include <cstdint>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/ca_nraw.hpp"
#include "custom_interfaces/msg/rad_status.hpp"
#include "rad.hpp"

using namespace custom_interfaces::msg;

class RAD_Wrist_Init : public rclcpp::Node
{
public:
    RAD_Wrist_Init();
    bool is_roll_calibrated();
    bool is_pitch_calibrated();

private:

    void _ls_callback(const RadStatus& msg);
    void _rs_callback(const RadStatus& msg);

    std::shared_ptr<rclcpp::Subscription<RadStatus> > left_wrist_sub, right_wrist_sub;

    bool left_ls, right_ls;
};