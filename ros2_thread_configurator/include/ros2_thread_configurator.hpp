#pragma once
#include "rclcpp/rclcpp.hpp"

namespace ros2_thread_configurator {

std::string create_callback_group_id(const rclcpp::CallbackGroup::SharedPtr &group, const rclcpp::Node::SharedPtr &node);

}

