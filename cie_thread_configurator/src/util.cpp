#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "cie_config_msgs/msg/callback_group_info.hpp"
#include "rclcpp/rclcpp.hpp"

#include "cie_thread_configurator/cie_thread_configurator.hpp"

namespace cie_thread_configurator {

std::string create_callback_group_id(
    rclcpp::CallbackGroup::SharedPtr group,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node) {
  std::stringstream ss;

  std::string ns = std::string(node->get_namespace());
  if (ns != "/")
    ns = ns + "/";

  ss << ns << node->get_name() << "@";

  auto sub_func = [&ss](const rclcpp::SubscriptionBase::SharedPtr &sub) {
    ss << "Subscription(" << sub->get_topic_name() << ")@";
  };

  auto service_func = [&ss](const rclcpp::ServiceBase::SharedPtr &service) {
    ss << "Service(" << service->get_service_name() << ")@";
  };

  auto client_func = [&ss](const rclcpp::ClientBase::SharedPtr &client) {
    ss << "Client(" << client->get_service_name() << ")@";
  };

  auto timer_func = [&ss](const rclcpp::TimerBase::SharedPtr &timer) {
    std::shared_ptr<const rcl_timer_t> timer_handle = timer->get_timer_handle();
    int64_t period;
    rcl_ret_t ret = rcl_timer_get_period(timer_handle.get(), &period);
    (void)ret;

    ss << "Timer(" << period << ")@";
  };

  auto waitable_func = [&ss](const rclcpp::Waitable::SharedPtr &waitable) {
    (void)waitable;
    ss << "Waitable"
       << "@";
  };

  group->collect_all_ptrs(sub_func, service_func, client_func, timer_func,
                          waitable_func);

  std::string ret = ss.str();
  ret.pop_back();

  return ret;
}

std::string create_callback_group_id(rclcpp::CallbackGroup::SharedPtr group,
                                     rclcpp::Node::SharedPtr node) {
  return create_callback_group_id(group, node->get_node_base_interface());
}

rclcpp::Publisher<cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr
create_client_publisher() {
  static int idx = 1;

  auto node = std::make_shared<rclcpp::Node>(
      "client_node" + std::to_string(idx++), "/cie_thread_configurator");
  auto publisher =
      node->create_publisher<cie_config_msgs::msg::CallbackGroupInfo>(
          "/cie_thread_configurator/callback_group_info",
          rclcpp::QoS(1000).keep_all());
  return publisher;
}

void publish_callback_group_info(
    const rclcpp::Publisher<cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr
        &publisher,
    int64_t tid, const std::string &callback_group_id) {
  auto message = std::make_shared<cie_config_msgs::msg::CallbackGroupInfo>();

  message->thread_id = tid;
  message->callback_group_id = callback_group_id;

  publisher->publish(*message);
  rclcpp::sleep_for(std::chrono::milliseconds(500));
}

} // namespace cie_thread_configurator
