#include <memory>
#include <cstdlib>
#include <string>
#include <iostream>
#include <sys/syscall.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"

#include "ros2_thread_configurator.hpp"

namespace rclcpp_components
{

class ComponentManagerRt : public rclcpp_components::ComponentManager {

public:
  template<typename... Args>
  ComponentManagerRt(Args&&... args) : rclcpp_components::ComponentManager(std::forward<Args>(args)...) {
    client_publisher_ = ros2_thread_configurator::create_client_publisher();

    auto tid = syscall(SYS_gettid);
    for (int i = 0; i < 3; i++) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      std::string node_id = this->get_name();

      ros2_thread_configurator::publish_callback_group_info(client_publisher_, tid, node_id + "@");
    }
  }

  ~ComponentManagerRt(){};
private:
  void passRt();
  rclcpp::Publisher<thread_config_msgs::msg::CallbackGroupInfo>::SharedPtr client_publisher_;
};
};  // namespace rclcpp_components

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto node = std::make_shared<rclcpp_components::ComponentManagerRt>();
  exec->add_node(node);
  exec->spin();
  rclcpp::shutdown();
  return 0;
}
