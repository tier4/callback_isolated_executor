#include <string>
#include <unordered_map>

#include <error.h>
#include <sched.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>

#include "yaml-cpp/yaml.h"
#include "rclcpp/rclcpp.hpp"

#include "thread_config_msgs/msg/callback_group_info.hpp"
#include "thread_configurator_node.hpp"

ThreadConfiguratorNode::ThreadConfiguratorNode(const YAML::Node &yaml) : Node("thread_configurator_node"), unapplied_num_(0) {
  {
    YAML::Node callback_groups = yaml["callback_groups"];
    unapplied_num_ = callback_groups.size();
    callback_group_configs_.resize(callback_groups.size());

    for (size_t i = 0; i < callback_groups.size(); i++) {
      const auto &callback_group = callback_groups[i];
      auto &config = callback_group_configs_[i];

      config.callback_group_id = callback_group["id"].as<std::string>();
      for (auto &cpu : callback_group["affinity"]) config.affinity.push_back(cpu.as<int>());
      config.policy = callback_group["policy"].as<std::string>();
      config.priority = callback_group["priority"].as<int>();

      id_to_callback_group_config_[config.callback_group_id] = &config;
    }
  }

  subscription_ = this->create_subscription<thread_config_msgs::msg::CallbackGroupInfo>(
    "/ros2_thread_configurator/callback_group_info", 100, std::bind(&ThreadConfiguratorNode::topic_callback, this, std::placeholders::_1));
}

bool ThreadConfiguratorNode::all_applied() {
  return unapplied_num_ == 0;
}

void ThreadConfiguratorNode::print_all_unapplied() {
  RCLCPP_WARN(this->get_logger(), "Following callback grouds are not yet configured");

  for (auto &config : callback_group_configs_) {
    if (!config.applied) {
      RCLCPP_WARN(this->get_logger(), "  - %s", config.callback_group_id.c_str());
    }
  }
}

bool ThreadConfiguratorNode::issue_syscalls(const CallbackGroupConfig &config) const {
  if (config.affinity.size() > 0) {
    cpu_set_t set;
    CPU_ZERO(&set);
    for (int cpu : config.affinity) CPU_SET(cpu, &set);
    if (sched_setaffinity(config.thread_id, sizeof(set), &set) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to configure affinity (id=%s, tid=%ld): %s",
          config.callback_group_id.c_str(), config.thread_id, strerror(errno));
      return false;
    }
  }

  if (config.policy == "SCHED_OTHER" || config.policy == "SCHED_BATCH" || config.policy == "SCHED_IDLE") {
    struct sched_param param;
    param.sched_priority = 0;

    static std::unordered_map<std::string, int> m = {
      {"SCHED_OTHER", SCHED_OTHER},
      {"SCHED_BATCH", SCHED_BATCH},
      {"SCHED_IDLE", SCHED_IDLE},
    };

    if (sched_setscheduler(config.thread_id, m[config.policy], &param) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to configure policy (id=%s, tid=%ld): %s",
          config.callback_group_id.c_str(), config.thread_id, strerror(errno));
      return false;
    }

    // Specify nice value
    if (setpriority(PRIO_PROCESS, config.thread_id, config.priority) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to configure nice value (id=%s, tid=%ld): %s",
          config.callback_group_id.c_str(), config.thread_id, strerror(errno));
      return false;
    }

  } else if (config.policy == "SCHED_FIFO" || config.policy == "SCHED_RR") {
    struct sched_param param;
    param.sched_priority = config.priority;

    static std::unordered_map<std::string, int> m = {
      {"SCHED_FIFO", SCHED_FIFO},
      {"SCHED_RR", SCHED_RR},
    };

    if (sched_setscheduler(config.thread_id, m[config.policy], &param) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to configure policy (id=%s, tid=%ld): %s",
          config.callback_group_id.c_str(), config.thread_id, strerror(errno));
      return false;
    }
  }

  return true;
}

void ThreadConfiguratorNode::topic_callback(const thread_config_msgs::msg::CallbackGroupInfo::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received CallbackGroupInfo: tid=%ld | %s", msg->thread_id, msg->callback_group_id.c_str());

  auto it = id_to_callback_group_config_.find(msg->callback_group_id);
  if (it == id_to_callback_group_config_.end()) {
    RCLCPP_WARN(this->get_logger(), "A config entry with the specified callback group id is not found (id=%s, tid=%ld)",
        msg->callback_group_id.c_str(), msg->thread_id);
    return;
  }

  CallbackGroupConfig *config = it->second;
  if (config->applied) {
    RCLCPP_WARN(this->get_logger(), "This callback group is already configured. skip (id=%s)", msg->callback_group_id.c_str());
    return;
  }

  config->thread_id = msg->thread_id;
  bool success = issue_syscalls(*config);

  if (!success) return;

  config->applied = true;
  unapplied_num_--;
}

