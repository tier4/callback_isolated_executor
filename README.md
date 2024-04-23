# callback_isolated_executor
The ComponentContainer and Executor that assign a dedicated thread for each callback group.

## Build and Install
```
$ git clone https://github.com/sykwer/callback_isolated_executor.git
$ cd callback_isolated_executor
$ source /opt/ros/humble/setup.bash
$ colcon build
$ source install/setup.bash
```

Set capability for the configurator executable to issue the syscalls like `sched_setscheduler(2)`.
```
$ sudo setcap cap_sys_nice+ep ./build/ros2_thread_configurator/thread_configurator_node
```

After elevating the priviridge level, part of dynamic linking functionality gets disabled for the security reason.
To deal with it, add a file with the following content under the `/etc/ld.so.conf.d/` directory.
The file name has to be `*.conf`.

```
/opt/ros/humble/lib
/opt/ros/humble/lib/x86_64-linux-gnu
/path/to/callback_isolated_executor/install/thread_config_msgs/lib
```

To enable the configuration, type the command below.
```
$ sudo ldconfig
```

## Usage
### Step1: Rewrite your app
If you are launching a node directly from the main function without using a ComponentContainer, change the name of the Executor.
```cpp
#include "static_callback_isolated_executor.hpp"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SampleNode>();
  auto executor = std::make_shared<StaticCallbackIsolatedExecutor>();

  executor->add_node(node);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}
```
