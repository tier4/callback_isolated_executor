#pragma once

#include <filesystem>
#include <set>
#include <string>

#include "yaml-cpp/yaml.h"
#include "rclcpp/rclcpp.hpp"

#include "thread_config_msgs/msg/callback_group_info.hpp"

class PrerunNode : public rclcpp::Node {
public:
  PrerunNode();
  void dump_yaml_config(std::filesystem::path path);

private:
  void topic_callback(const thread_config_msgs::msg::CallbackGroupInfo::SharedPtr msg);

  rclcpp::Subscription<thread_config_msgs::msg::CallbackGroupInfo>::SharedPtr subscription_;
  std::set<std::string> callback_group_ids_;
};

