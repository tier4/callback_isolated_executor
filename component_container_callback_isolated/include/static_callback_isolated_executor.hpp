#pragma once
#include "rclcpp/rclcpp.hpp"

class StaticCallbackIsolatedExecutor {
public:
  void add_node(const rclcpp::Node::SharedPtr &node);
  void spin();
private:
  rclcpp::Node::SharedPtr node_;
};

