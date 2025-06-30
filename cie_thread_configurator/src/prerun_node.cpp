#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "cie_config_msgs/msg/callback_group_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "cie_thread_configurator/prerun_node.hpp"

PrerunNode::PrerunNode() : Node("prerun_node") {
  subscription_ =
      this->create_subscription<cie_config_msgs::msg::CallbackGroupInfo>(
          "/cie_thread_configurator/callback_group_info", 100,
          std::bind(&PrerunNode::topic_callback, this, std::placeholders::_1));
}

void PrerunNode::topic_callback(
    const cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg) {
  if (callback_group_ids_.find(msg->callback_group_id) !=
      callback_group_ids_.end())
    return;

  RCLCPP_INFO(this->get_logger(), "Received CallbackGroupInfo: tid=%ld | %s",
              msg->thread_id, msg->callback_group_id.c_str());

  callback_group_ids_.insert(msg->callback_group_id);
}

void PrerunNode::dump_yaml_config(std::filesystem::path path) {
  YAML::Emitter out;

  out << YAML::BeginMap;
  out << YAML::Key << "callback_groups";
  out << YAML::Value << YAML::BeginSeq;

  for (const std::string &callback_group_id : callback_group_ids_) {
    out << YAML::BeginMap;
    out << YAML::Key << "id" << YAML::Value << callback_group_id;
    out << YAML::Key << "affinity" << YAML::Value << YAML::Null;
    out << YAML::Key << "policy" << YAML::Value << "SCHED_OTHER";
    out << YAML::Key << "priority" << YAML::Value << 0;
    out << YAML::EndMap;
    out << YAML::Newline;
  }

  out << YAML::EndSeq;
  out << YAML::EndMap;

  std::ofstream fout(path / "template.yaml");
  fout << out.c_str();
  fout.close();

  std::cout << "template.yaml is created in the current directory" << std::endl;
}
