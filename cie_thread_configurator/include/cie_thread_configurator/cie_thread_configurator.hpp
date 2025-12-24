#pragma once

#include "rclcpp/rclcpp.hpp"
#include <map>
#include <memory>
#include <string>
#include <sys/syscall.h>

#include "cie_config_msgs/msg/callback_group_info.hpp"

namespace cie_thread_configurator {

std::string create_callback_group_id(rclcpp::CallbackGroup::SharedPtr group,
                                     rclcpp::Node::SharedPtr node);

std::string create_callback_group_id(
    rclcpp::CallbackGroup::SharedPtr group,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node);

// Caution: Do not call in parallel
// Caution: Must be called after rclcpp::init() called
rclcpp::Publisher<cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr
create_client_publisher();

// `publisher` is assumed to be the return value of create_client_publisher()
void publish_callback_group_info(
    const rclcpp::Publisher<cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr
        &publisher,
    int64_t tid, const std::string &callback_group_id);

// Get hardware information from lscpu command
std::map<std::string, std::string> get_hardware_info();

/// Spawn a thread whose scheduling policy can be managed through
/// cie_thread_configurator.
/// Caution: the `thread_name` must be unique among threads managed by
/// cie_thread_configurator.
template <class F, class... Args>
std::thread spawn_cie_thread(const char *thread_name, F &&f, Args &&...args) {
  std::thread t(
      [thread_name, func = std::forward<F>(f),
       captured_args = std::make_tuple(std::forward<Args>(args)...)]() mutable {
        // Create a unique rclcpp context in case `rclcpp::init()` is not called
        rclcpp::InitOptions init_options;
        init_options.shutdown_on_signal = false;
        init_options.auto_initialize_logging(false);
        auto context = std::make_shared<rclcpp::Context>();
        context->init(0, nullptr, init_options);

        rclcpp::NodeOptions options;
        options.context(context);
        auto node = std::make_shared<rclcpp::Node>(
            "cie_thread_client", "/cie_thread_configurator", options);

        auto publisher =
            node->create_publisher<cie_config_msgs::msg::CallbackGroupInfo>(
                "/cie_thread_configurator/callback_group_info",
                rclcpp::QoS(1000).keep_all());
        auto tid = static_cast<pid_t>(syscall(SYS_gettid));
        publish_callback_group_info(publisher, tid, thread_name);

        context->shutdown("Publishing is finished.");

        std::apply(std::move(func), std::move(captured_args));
      });

  return t;
}

} // namespace cie_thread_configurator
