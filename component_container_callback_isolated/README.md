# component_container_callback_isolated
A component container that assigns a dedicated thread for each callback group.

## Build and Install
`component_container_calblack_isolated` depends on https://github.com/sykwer/ros2_thread_configurator.
```
$ mkdir -p /path/to/ros2_thread_configurator_ws
$ cd /path/to/ros2_thread_configurator_ws
$ git clone https://github.com/sykwer/ros2_thread_configurator.git src/
$ source /opt/ros/humble/setup.bash
$ colcon build
```
After `ros2_thread_configurator` is installed, we can build `component_container_callback_isolated`.
```
$ mkdir -p /path/to/component_container_callback_isolated_ws
$ cd /path/to/component_container_callback_isolated_ws
$ git clone https://github.com/sykwer/component_container_callback_isolated.git src/
$ source /path/to/ros2_thread_configurator_ws/install/setup.bash
$ colcon build
$ source install/setup.bash
```

## How to Use
Refer to the [launch directory](https://github.com/sykwer/component_container_callback_isolated/tree/main/launch).

## Quick Demo
Demonstration of launching the component container and loading the sample node at the same time.
```
$ ros2 launch rclcpp_component_container_callback_isolated sample_node.launch.xml
```
Demonstration of loading the sample node into the pre-launched component container afterwards.
```
$ ros2 run rclcpp_component_container_callback_isolated component_container_callback_isolated --ros-args --remap __node:=sample_container
$ ros2 launch rclcpp_component_container_callback_isolated load_sample_node.launch.xml
```
