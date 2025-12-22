#include "callback_isolated_executor/callback_isolated_executor.hpp"
#include "cie_sample_application/reentrant_node.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ReentrantNode>();
  auto executor = std::make_shared<CallbackIsolatedExecutor>();
  // executor->set_reentrant_parallelism(4);

  executor->add_node(node);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}
