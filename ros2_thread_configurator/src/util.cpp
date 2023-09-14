#include <functional>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "thread_configurator.hpp"

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

