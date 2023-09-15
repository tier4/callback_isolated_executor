#pragma once

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "thread_config_msgs/msg/callback_group_info.hpp"

namespace ros2_thread_configurator {

std::string create_callback_group_id(const rclcpp::CallbackGroup::SharedPtr &group, const rclcpp::Node::SharedPtr &node);

// Caution: Do not call in parallel
// Caution: Must be called after rclcpp::init() called
rclcpp::Publisher<thread_config_msgs::msg::CallbackGroupInfo>::SharedPtr create_client_publisher();

// `publisher` is assumed to be the return value of create_client_publisher()
void publish_callback_group_info(const rclcpp::Publisher<thread_config_msgs::msg::CallbackGroupInfo>::SharedPtr &publisher,
    int64_t tid, const std::string &callback_group_id);

}

