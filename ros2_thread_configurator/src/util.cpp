#include <chrono>
#include <functional>
#include <string>
#include <sstream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "thread_config_msgs/msg/callback_group_info.hpp"

#include "ros2_thread_configurator.hpp"

namespace ros2_thread_configurator {

std::string create_callback_group_id(const rclcpp::CallbackGroup::SharedPtr &group, const rclcpp::Node::SharedPtr &node) {
  std::stringstream ss;

  ss << node->get_namespace() << "/" << node->get_name() << "@";

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
    (void) ret;

    ss << "Timer(" << period << ")@";
  };

  auto waitable_func = [&ss](const rclcpp::Waitable::SharedPtr &waitable) {
    (void) waitable;
    ss << "Waitable" << "@";
  };

  group->collect_all_ptrs(sub_func, service_func, client_func, timer_func, waitable_func);

  std::string ret = ss.str();
  ret.pop_back();

  return ret;
}

rclcpp::Publisher<thread_config_msgs::msg::CallbackGroupInfo>::SharedPtr create_client_publisher() {
  auto node = std::make_shared<rclcpp::Node>("client_node", "/ros2_thread_configurator");
  auto publisher = node->create_publisher<thread_config_msgs::msg::CallbackGroupInfo>("/ros2_thread_configurator/callback_group_info", 10);
  return publisher;
}

void publish_callback_group_info(const rclcpp::Publisher<thread_config_msgs::msg::CallbackGroupInfo>::SharedPtr &publisher,
    int64_t tid, const std::string &callback_group_id) {
  auto message = std::make_shared<thread_config_msgs::msg::CallbackGroupInfo>();

  message->thread_id = tid;
  message->callback_group_id = callback_group_id;

  publisher->publish(*message);
  rclcpp::sleep_for(std::chrono::milliseconds(500));
}

} // namespace ros2_thread_configurator
