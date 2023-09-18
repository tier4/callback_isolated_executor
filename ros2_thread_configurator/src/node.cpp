#include <memory>
#include <string>
#include <unordered_map>

#include "yaml-cpp/yaml.h"
#include "rclcpp/rclcpp.hpp"

#include "thread_config_msgs/msg/callback_group_info.hpp"

using std::placeholders::_1;

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
  ThreadConfiguratorNode(const YAML::Node &yaml) : Node("thread_configurator_node"), unapplied_num_(0) {
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
      "/ros2_thread_configurator/callback_group_info", 100, std::bind(&ThreadConfiguratorNode::topic_callback, this, _1));
  }

private:
  bool issue_syscalls(const CallbackGroupConfig &config) const {
    (void) config;
    // config.affinity
    // config.policy
    // config.priority
    return true;
  }

  void topic_callback(const thread_config_msgs::msg::CallbackGroupInfo::SharedPtr msg) {
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

    if (success && !config->applied) {
      config->applied = true;
      unapplied_num_--;
    }
  }

  rclcpp::Subscription<thread_config_msgs::msg::CallbackGroupInfo>::SharedPtr subscription_;

  std::vector<CallbackGroupConfig> callback_group_configs_;
  std::unordered_map<std::string, CallbackGroupConfig*> id_to_callback_group_config_;
  int unapplied_num_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  bool next = false;
  std::string filename;

  std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);

  for (auto &arg : args) {
    if (next) {
      filename = arg;
      break;
    }

    if (arg == std::string("--config-file")) next = true;
  }

  YAML::Node config;

  try {
    config = YAML::LoadFile(filename);
    std::cout << config << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Error reading the YAML file: " << e.what() << std::endl;
  }

  rclcpp::spin(std::make_shared<ThreadConfiguratorNode>(config));
  rclcpp::shutdown();
  return 0;
}
