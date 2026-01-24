#include <iostream>

#include "cie_thread_configurator/cie_thread_configurator.hpp"

void worker_function() { std::cout << "Worker thread running" << std::endl; }

int main(int /*argc*/, char ** /*argv*/) {
  auto thread = cie_thread_configurator::spawn_non_ros2_thread(
      "standalone_worker", worker_function);
  thread.join();
  return 0;
}
