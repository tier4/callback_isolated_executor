#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class ReentrantNode : public rclcpp::Node {
public:
  explicit ReentrantNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void timer_callback_1();
  void timer_callback_2();
  void subscription_callback(const std_msgs::msg::Int32::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::TimerBase::SharedPtr timer2_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;

  // Single reentrant group to allow parallel callbacks
  rclcpp::CallbackGroup::SharedPtr reentrant_group_;

  std::atomic<int> count_{0};
};
