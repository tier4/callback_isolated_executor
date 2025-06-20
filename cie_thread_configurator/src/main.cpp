#include <filesystem>
#include <memory>
#include <string>

#include "yaml-cpp/yaml.h"
#include "rclcpp/rclcpp.hpp"

#include "prerun_node.hpp"
#include "thread_configurator_node.hpp"

static void spin_thread_configurator_node(const std::string &config_filename) {
  YAML::Node config;

  try {
    config = YAML::LoadFile(config_filename);
    std::cout << config << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Error reading the YAML file: " << e.what() << std::endl;
    return;
  }

  auto node = std::make_shared<ThreadConfiguratorNode>(config);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  executor->add_node(node);

  while (rclcpp::ok() && !node->all_applied()) {
    executor->spin_once();
  }

  if (node->all_applied()) {
    RCLCPP_INFO(node->get_logger(), "Apply sched deadline?");
    std::cin.get();

    node->apply_deadline_configs();

    RCLCPP_INFO(node->get_logger(),
                "Success: All of the configurations are applied."
                "\nPress enter to exit and remove cgroups, if there are SCHED_DEADLINE tasks:");
    std::cin.get();
  } else {
    node->print_all_unapplied();
  }
}

static void spin_prerun_node() {
  std::cout << "prerun mode" << std::endl;

  auto node = std::make_shared<PrerunNode>();
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  executor->add_node(node);
  executor->spin();

  node->dump_yaml_config(std::filesystem::current_path());
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);

  bool prerun_mode = false;
  for (auto &arg : args) if (arg == "--prerun") {
    prerun_mode = true;
  }

  if (prerun_mode) {
    spin_prerun_node();
  } else {
    bool next = false;
    std::string filename;

    for (auto &arg : args) {
      if (next) {
        filename = arg;
        break;
      }

      if (arg == std::string("--config-file")) next = true;
    }

    spin_thread_configurator_node(filename);
  }

  rclcpp::shutdown();
  return 0;
}
