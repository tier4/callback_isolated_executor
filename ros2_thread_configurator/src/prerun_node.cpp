#include <filesystem>
#include <fstream>
#include <iostream>
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

  callback_group_ids_.push_back(msg->callback_group_id);
}

void PrerunNode::dump_yaml_config(std::filesystem::path path) {
  YAML::Node yaml;
  YAML::Node callback_groups;
  yaml["callback_groups"] = callback_groups;

  for (const std::string &callback_group_id : callback_group_ids_) {
    YAML::Node callback_group;
    callback_group["id"] = callback_group_id;

    YAML::Node affinity;
    affinity.push_back(0);
    affinity.push_back(1);
    callback_group["affinity"] = affinity;

    callback_group["policy"] = "SCHED_OTHER";
    callback_group["priority"] = 0;

    callback_groups.push_back(callback_group);
  }

  std::ofstream fout(path / "template.yaml");
  fout << yaml;
  fout.close();

  RCLCPP_INFO(this->get_logger(), "template.yaml is created in the current directory");
}

