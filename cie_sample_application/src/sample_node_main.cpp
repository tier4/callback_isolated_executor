#include "callback_isolated_executor/callback_isolated_executor.hpp"
#include "cie_sample_application/sample_node.hpp"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SampleNode>();
  auto executor = std::make_shared<StaticCallbackIsolatedExecutor>();

  executor->add_node(node);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}
