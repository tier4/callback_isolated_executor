#include <chrono>
#include <memory>
#include <sys/syscall.h>

#include "cie_sample_application/sample_node.hpp"
#include "cie_thread_configurator/cie_thread_configurator.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;
const long long MESSAGE_SIZE = 1024;

SampleNode::SampleNode(const rclcpp::NodeOptions &options)
    : Node("sample_node", "/sample_space/sample_subspace", options), count_(0),
      count2_(0) {
  publisher_ = this->create_publisher<std_msgs::msg::Int32>("topic_out", 1);
  publisher2_ = this->create_publisher<std_msgs::msg::Int32>("topic_out2", 1);

  group1_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  group2_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  group3_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  timer_ = this->create_wall_timer(
      3000ms, std::bind(&SampleNode::timer_callback, this), group1_);
  timer2_ = this->create_wall_timer(
      1333ms, std::bind(&SampleNode::timer_callback2, this), group2_);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = group3_;

  subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "topic_in", 1,
      std::bind(&SampleNode::subscription_callback, this,
                std::placeholders::_1),
      sub_options);

  non_ros_thread_ = cie_thread_configurator::spawn_non_ros2_thread(
      "sample_non_ros_thread", &SampleNode::non_ros_thread_func, this, 42);
}

SampleNode::~SampleNode() {
  if (non_ros_thread_.joinable()) {
    non_ros_thread_.join();
  }
}

void SampleNode::timer_callback() {
  long tid = syscall(SYS_gettid);
  auto message = std_msgs::msg::Int32();
  message.data = count_++;
  RCLCPP_INFO(this->get_logger(),
              "Publishing Message ID (timer_callback tid=%ld): '%d'", tid,
              message.data);
  publisher_->publish(std::move(message));
}

void SampleNode::timer_callback2() {
  long tid = syscall(SYS_gettid);
  auto message = std_msgs::msg::Int32();
  message.data = count2_++;
  RCLCPP_INFO(this->get_logger(),
              "Publishing Message ID (timer_callback2 tid=%ld): '%d'", tid,
              message.data);
  publisher2_->publish(std::move(message));
}

void SampleNode::subscription_callback(
    const std_msgs::msg::Int32::SharedPtr msg) {
  long tid = syscall(SYS_gettid);
  RCLCPP_INFO(this->get_logger(),
              "I heard message ID (subscription_callback tid=%ld): '%d'", tid,
              msg->data);
}

void SampleNode::non_ros_thread_func(int value) {
  long tid = syscall(SYS_gettid);
  for (int i = 0; i < 5; ++i) {
    std::this_thread::sleep_for(2s);
    RCLCPP_INFO(this->get_logger(), "Test thread running (tid=%ld), value: %d",
                tid, value);
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(SampleNode)
