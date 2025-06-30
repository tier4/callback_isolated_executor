#pragma once

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <string>

#include "cie_config_msgs/msg/callback_group_info.hpp"

namespace cie_thread_configurator {

std::string create_callback_group_id(rclcpp::CallbackGroup::SharedPtr group,
                                     rclcpp::Node::SharedPtr node);

std::string create_callback_group_id(
    rclcpp::CallbackGroup::SharedPtr group,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node);

// Caution: Do not call in parallel
// Caution: Must be called after rclcpp::init() called
rclcpp::Publisher<cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr
create_client_publisher();

// `publisher` is assumed to be the return value of create_client_publisher()
void publish_callback_group_info(
    const rclcpp::Publisher<cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr
        &publisher,
    int64_t tid, const std::string &callback_group_id);

} // namespace cie_thread_configurator
