#pragma once
#include <cstdint>
#include <vector>
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

    std::vector<int64_t> rad_ids;
    size_t num_of_rads;

private:
    std::vector<std::string> rad_status;

    void _callback(const RadStatus& msg, int id);

    std::vector<std::shared_ptr<rclcpp::Subscription<RadStatus>>> sub;
    std::vector<bool> ls;

};