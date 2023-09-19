#include <string>

#include "yaml-cpp/yaml.h"
#include "rclcpp/rclcpp.hpp"

#include "thread_config_msgs/msg/callback_group_info.hpp"
#include "prerun_node.hpp"

PrerunNode::PrerunNode() : Node("prerun_node") {
  subscription_ = this->create_subscription<thread_config_msgs::msg::CallbackGroupInfo>(
    "/ros2_thread_configurator/callback_group_info", 100, std::bind(&PrerunNode::topic_callback, this, std::placeholders::_1));
}

void PrerunNode::topic_callback(const thread_config_msgs::msg::CallbackGroupInfo::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received CallbackGroupInfo: tid=%ld | %s", msg->thread_id, msg->callback_group_id.c_str());
}

