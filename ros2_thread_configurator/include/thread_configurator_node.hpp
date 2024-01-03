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

    // For SCHED_DEADLINE
    unsigned int runtime;
    unsigned int period;
    unsigned int deadline;

    bool applied = false;
  };

public:
  ThreadConfiguratorNode(const YAML::Node &yaml);
  ~ThreadConfiguratorNode();
  bool all_applied();
  void print_all_unapplied();

private:
  bool set_affinity_by_cgroup(int64_t thread_id, const std::vector<int>& cpus);
  bool issue_syscalls(const CallbackGroupConfig &config);
  void topic_callback(const thread_config_msgs::msg::CallbackGroupInfo::SharedPtr msg);

  rclcpp::Subscription<thread_config_msgs::msg::CallbackGroupInfo>::SharedPtr subscription_;

  std::vector<CallbackGroupConfig> callback_group_configs_;
  std::unordered_map<std::string, CallbackGroupConfig*> id_to_callback_group_config_;
  int unapplied_num_;
  int cgroup_num_;
};

