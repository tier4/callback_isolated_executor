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

class ComponentManagerNode : public rclcpp_components::ComponentManager {

public:
  template<typename... Args>
  ComponentManagerNode(Args&&... args) : rclcpp_components::ComponentManager(std::forward<Args>(args)...) {
    client_publisher_ = ros2_thread_configurator::create_client_publisher();
  }

  ~ComponentManagerNode(){};

protected:
  void add_node_to_executor(uint64_t node_id) override;
private:
  rclcpp::Publisher<thread_config_msgs::msg::CallbackGroupInfo>::SharedPtr client_publisher_;
};


void ComponentManagerNode::add_node_to_executor(uint64_t node_id){
  auto node = node_wrappers_[node_id].get_node_base_interface();
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);

  auto tid = syscall(SYS_gettid);
  for (int i = 0; i < 3; i++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ros2_thread_configurator::publish_callback_group_info(this->client_publisher_, tid, ros2_thread_configurator::create_node_id(node));
  }

}

} //rclcpp_components name space

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto node = std::make_shared<rclcpp_components::ComponentManagerNode>();
  exec->add_node(node);
  exec->spin();
}