# cie_sample_application
A sample application to demonstrate the Executors and ComponentContainers in the `callback_isolated_executor` package.

## Launch without ComponentContainer (from main function)
```bash
$ ros2 run cie_sample_application sample_node_main
```

## Launch with ComponentContainer
Launch the component container and load the node at the same time.
```bash
$ ros2 launch cie_sample_application sample_node.launch.xml
```

Or, load the node to the exsiting component container.
```bash
$ ros2 run callback_isolated_executor component_container_callback_isolated --ros-args --remap __node:=sample_container
$ ros2 launch cie_sample_application load_sample_node.launch.xml
```

## Standalone Non-ROS Process
```bash
$ ros2 run cie_sample_application sample_non_ros_process
```

This demonstrates using `spawn_cie_thread` in a standalone process without any ROS2 node.
