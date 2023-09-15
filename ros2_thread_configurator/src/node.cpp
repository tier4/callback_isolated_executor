#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "thread_config_msgs/msg/callback_group_info.hpp"

using std::placeholders::_1;

class ThreadConfiguratorNode : public rclcpp::Node {
public:
  ThreadConfiguratorNode() : Node("thread_configurator_node") {
    subscription_ = this->create_subscription<thread_config_msgs::msg::CallbackGroupInfo>(
      "/ros2_thread_configurator/callback_group_info", 10, std::bind(&ThreadConfiguratorNode::topic_callback, this, _1));
  }

private:
  void topic_callback(const thread_config_msgs::msg::CallbackGroupInfo::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "tid=%ld | %s", msg->thread_id, msg->callback_group_id.c_str());
  }

  rclcpp::Subscription<thread_config_msgs::msg::CallbackGroupInfo>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThreadConfiguratorNode>());
  rclcpp::shutdown();
  return 0;
}
