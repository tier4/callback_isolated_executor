#include <chrono>
#include <memory>
#include <sys/syscall.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/int32.hpp"

#include "static_callback_isolated_executor.hpp"

using namespace std::chrono_literals;

const long long MESSAGE_SIZE = 1024;

class SampleNode : public rclcpp::Node {
public:
  explicit SampleNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node("sample_node", "/sample_space/sample_subspace", options), count_(0), count2_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("topic_out", 1);
    publisher2_ = this->create_publisher<std_msgs::msg::Int32>("topic_out2", 1);

    group1_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    group2_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    group3_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    timer_ = this->create_wall_timer(3000ms, std::bind(&SampleNode::timer_callback, this), group1_);
    timer2_ = this->create_wall_timer(1333ms, std::bind(&SampleNode::timer_callback2, this), group2_);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = group3_;

    subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "topic_in", 1, std::bind(&SampleNode::subscription_callback, this, std::placeholders::_1), sub_options);
  }

private:
  void timer_callback() {
    long tid = syscall(SYS_gettid);
    auto message = std_msgs::msg::Int32();
    message.data = count_++;
    RCLCPP_INFO(this->get_logger(), "Publishing Message ID (timer_callback tid=%ld): '%d'", tid, message.data);
    publisher_->publish(std::move(message));
  }

  void timer_callback2() {
    long tid = syscall(SYS_gettid);
    auto message = std_msgs::msg::Int32();
    message.data = count2_++;
    RCLCPP_INFO(this->get_logger(), "Publishing Message ID (timer_callback2 tid=%ld): '%d'", tid, message.data);
    publisher2_->publish(std::move(message));
  }

  void subscription_callback(const std_msgs::msg::Int32::SharedPtr msg) const {
    long tid = syscall(SYS_gettid);
    RCLCPP_INFO(this->get_logger(), "I heard message ID (subscription_callback tid=%ld): '%d'", tid, msg->data);
  }

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

RCLCPP_COMPONENTS_REGISTER_NODE(SampleNode)

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SampleNode>();
  auto executor = std::make_shared<StaticCallbackIsolatedExecutor>();

  executor->add_node(node);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}
