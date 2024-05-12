#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class SampleNode : public rclcpp::Node {
public:
  explicit SampleNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void timer_callback();
  void timer_callback2();
  void subscription_callback(const std_msgs::msg::Int32::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer2_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher2_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;

  // Need to be stored not to be destructed
  rclcpp::CallbackGroup::SharedPtr group1_;
  rclcpp::CallbackGroup::SharedPtr group2_;
  rclcpp::CallbackGroup::SharedPtr group3_;

  size_t count_;
  size_t count2_;
};