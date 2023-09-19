#pragma once

#include <string>
#include <unordered_map>

#include "yaml-cpp/yaml.h"
#include "rclcpp/rclcpp.hpp"

#include "thread_config_msgs/msg/callback_group_info.hpp"

class ThreadConfiguratorNode : public rclcpp::Node {
  struct CallbackGroupConfig {
    std::string callback_group_id;
    int64_t thread_id = -1;
    std::vector<int> affinity;
    std::string policy;
    int priority;
    bool applied = false;
  };

public:
  ThreadConfiguratorNode(const YAML::Node &yaml);
  bool all_applied();

private:
  bool issue_syscalls(const CallbackGroupConfig &config) const;
  void topic_callback(const thread_config_msgs::msg::CallbackGroupInfo::SharedPtr msg);

  rclcpp::Subscription<thread_config_msgs::msg::CallbackGroupInfo>::SharedPtr subscription_;

  std::vector<CallbackGroupConfig> callback_group_configs_;
  std::unordered_map<std::string, CallbackGroupConfig*> id_to_callback_group_config_;
  int unapplied_num_;
};

