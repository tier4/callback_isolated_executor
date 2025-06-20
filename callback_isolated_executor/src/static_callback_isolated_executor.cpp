#include <vector>
#include <thread>
#include <string>
#include <memory>
#include <sys/syscall.h>

#include "rclcpp/rclcpp.hpp"
#include "cie_thread_configurator.hpp"

#include "static_callback_isolated_executor.hpp"

void StaticCallbackIsolatedExecutor::add_node(const rclcpp::Node::SharedPtr &node) {
  node_ = node->get_node_base_interface();
}
void StaticCallbackIsolatedExecutor::add_node(const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr &node) {
  node_ = node;
}

void StaticCallbackIsolatedExecutor::spin() {
  std::vector<std::thread> threads;
  std::vector<std::string> callback_group_ids;

  node_->for_each_callback_group([this, &callback_group_ids](rclcpp::CallbackGroup::SharedPtr group) {
      if (group->get_associated_with_executor_atomic().load()) {
        std::string id = cie_thread_configurator::create_callback_group_id(group, node_);
        //RCLCPP_WARN(node_->get_logger(), "A callback group (%s) has been already added to an executor. skip.", id.c_str());
        return;
      }

      auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      //executor->add_callback_group(group, node_->get_node_base_interface());
      executor->add_callback_group(group, node_);
      executors.push_back(executor);
      callback_group_ids.push_back(cie_thread_configurator::create_callback_group_id(group, node_));
  });

  auto client_publisher = cie_thread_configurator::create_client_publisher();

  for (size_t i = 0; i < executors.size(); i++) {
    auto &executor = executors[i];
    auto &callback_group_id = callback_group_ids[i];

    threads.emplace_back([&executor, &callback_group_id, &client_publisher]() {
        auto tid = syscall(SYS_gettid);
        cie_thread_configurator::publish_callback_group_info(client_publisher, tid, callback_group_id);
        executor->spin();
    });
  }

  for (auto &t : threads) {
    t.join();
  }
}

void StaticCallbackIsolatedExecutor::remove_node(const rclcpp::Node::SharedPtr &node){
  this->remove_node(node->get_node_base_interface());
}

void StaticCallbackIsolatedExecutor::remove_node(const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr &node){
    //executors[0]->remove_node(node, false);
    for(auto &e : executors){
      e->remove_node(node, false);
    }
}
