#include <string>
#include <unordered_map>
#include <fstream>
#include <filesystem>

#include <error.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>

#include "yaml-cpp/yaml.h"
#include "rclcpp/rclcpp.hpp"

#include "thread_config_msgs/msg/callback_group_info.hpp"
#include "thread_configurator_node.hpp"
#include "sched_deadline.hpp"

ThreadConfiguratorNode::ThreadConfiguratorNode(const YAML::Node &yaml) : Node("thread_configurator_node"), unapplied_num_(0), cgroup_num_(0) {
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

      if (config.policy == "SCHED_DEADLINE") {
        config.runtime = callback_group["runtime"].as<unsigned int>();
        config.period = callback_group["period"].as<unsigned int>();
        config.deadline = callback_group["deadline"].as<unsigned int>();
      } else {
        config.priority = callback_group["priority"].as<int>();
      }

      if (config.policy == "SCHED_DEADLINE") {
        config.runtime = callback_group["runtime"].as<unsigned int>();
        config.period = callback_group["period"].as<unsigned int>();
        config.deadline = callback_group["deadline"].as<unsigned int>();
      }

      id_to_callback_group_config_[config.callback_group_id] = &config;
    }
  }

  subscription_ = this->create_subscription<thread_config_msgs::msg::CallbackGroupInfo>(
    "/ros2_thread_configurator/callback_group_info", rclcpp::QoS(1000).keep_all(), std::bind(&ThreadConfiguratorNode::topic_callback, this, std::placeholders::_1));
}

ThreadConfiguratorNode::~ThreadConfiguratorNode() {
  if (cgroup_num_ > 0) {
    for (int i = 0; i < cgroup_num_; i++) {
      rmdir(("/sys/fs/cgroup/cpuset/" + std::to_string(i)).c_str());
    }
  }
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

bool ThreadConfiguratorNode::set_affinity_by_cgroup(int64_t thread_id, const std::vector<int>& cpus) {
  std::string cgroup_path = "/sys/fs/cgroup/cpuset/" + std::to_string(cgroup_num_++);
  if (!std::filesystem::create_directory(cgroup_path)) {
    return false;
  }

  std::string cpus_path = cgroup_path + "/cpuset.cpus";
  if (std::ofstream cpus_file{cpus_path}) {
    for (int cpu : cpus) cpus_file << cpu << ",";
  } else {
    return false;
  }

  std::string mems_path = cgroup_path + "/cpuset.mems";
  if (std::ofstream mems_file{mems_path}) {
    mems_file << 0;
  } else {
    return false;
  }

  std::string tasks_path = cgroup_path + "/tasks";
  if (std::ofstream tasks_file{tasks_path}) {
    tasks_file << thread_id;
  } else {
    return false;
  }

  return true;
}

bool ThreadConfiguratorNode::issue_syscalls(const CallbackGroupConfig &config) {
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

  } else if (config.policy == "SCHED_DEADLINE") {
    struct sched_attr attr;
    attr.size = sizeof(attr);
    attr.sched_flags = 0;
    attr.sched_nice = 0;
    attr.sched_priority = 0;

    attr.sched_policy = SCHED_DEADLINE;
    attr.sched_runtime = config.runtime;
    attr.sched_period = config.period;
    attr.sched_deadline = config.deadline;

    if (sched_setattr(config.thread_id, &attr, 0) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to configure policy (id=%s, tid=%ld): %s",
          config.callback_group_id.c_str(), config.thread_id, strerror(errno));
      return false;
    }
  }

  if (config.affinity.size() > 0) {
    if (config.policy == "SCHED_DEADLINE") {
      if (!set_affinity_by_cgroup(config.thread_id, config.affinity)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to configure affinity (id=%s, tid=%ld): %s",
            config.callback_group_id.c_str(), config.thread_id, "Please disable cgroup v2 if used: `systemd.unified_cgroup_hierarchy=0`");
        return false;
        }

    } else {
      cpu_set_t set;
      CPU_ZERO(&set);
      for (int cpu : config.affinity) CPU_SET(cpu, &set);
      if (sched_setaffinity(config.thread_id, sizeof(set), &set) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to configure affinity (id=%s, tid=%ld): %s",
            config.callback_group_id.c_str(), config.thread_id, strerror(errno));
        return false;
      }
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

