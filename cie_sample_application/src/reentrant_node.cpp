#include <sys/syscall.h>
#include <thread>
#include <utility>

#include "cie_sample_application/reentrant_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

ReentrantNode::ReentrantNode(const rclcpp::NodeOptions &options)
    : Node("reentrant_node", options) {
  // Create a single reentrant callback group
  reentrant_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  timer1_ = this->create_wall_timer(100ms, std::bind(&ReentrantNode::timer_callback_1, this), reentrant_group_);
  timer2_ = this->create_wall_timer(1000ms, std::bind(&ReentrantNode::timer_callback_2, this), reentrant_group_);
}

void ReentrantNode::timer_callback_1() {
  long tid = syscall(SYS_gettid);
  RCLCPP_INFO(this->get_logger(), "Timer1 (T=100ms) (tid=%ld)", tid);
}

void ReentrantNode::timer_callback_2() {
  std::this_thread::sleep_for(900ms);
  long tid = syscall(SYS_gettid);
  RCLCPP_INFO(this->get_logger(), "Timer2 (T=1000ms) (tid=%ld)", tid);
}

RCLCPP_COMPONENTS_REGISTER_NODE(ReentrantNode)
