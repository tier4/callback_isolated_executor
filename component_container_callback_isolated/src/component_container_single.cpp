#include <memory>
#include <cstdlib>
#include <string>
#include <iostream>
#include <sys/syscall.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"

#include "ros2_thread_configurator.hpp"

int main(int argc, char * argv[])
{
  const char* ret = getenv("THREAD_CONFIGURATION_ID");
  if (ret == nullptr) {
    std::cerr << "THREAD_CONFIGURATION_ID environment variable is needed" << std::endl;
    return -1;
  }

  rclcpp::init(argc, argv);

  std::string config_id = ret;

  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto node = std::make_shared<rclcpp_components::ComponentManager>(exec);
  exec->add_node(node);

  auto config_publisher = node->create_publisher<thread_config_msgs::msg::CallbackGroupInfo>(
      "/ros2_thread_configurator/callback_group_info", rclcpp::QoS(1000).keep_all());
  auto tid = syscall(SYS_gettid);

  // dirty
  for (int i = 0; i < 10; i++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ros2_thread_configurator::publish_callback_group_info(config_publisher, tid, config_id);
  }

  exec->spin();
}
