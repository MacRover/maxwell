#pragma once
#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/ca_nraw.hpp"
#include "custom_interfaces/msg/rad_status.hpp"
#include "rad.hpp"

using namespace custom_interfaces::msg;

class RAD_Init : public rclcpp::Node
{
public:
    RAD_Init();
    bool finished();

private:
    void _callback_fr(const RadStatus& msg);
    void _callback_fl(const RadStatus& msg);
    void _callback_br(const RadStatus& msg);
    void _callback_bl(const RadStatus& msg);

    std::shared_ptr<rclcpp::Subscription<RadStatus> > sub_fr, 
        sub_fl, sub_br, sub_bl;

    bool fr_ls, fl_ls, br_ls, bl_ls;

};