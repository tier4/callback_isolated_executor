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
When running a node within `ComponentContainerCallbackIsolated`, you don't need to modify the node's implementation.
However, if starting the node directly from the main function without using ComponentContainer, you need to modify the node's implementation as shown below and rebuild it.

#### Option1: Launch without ComponentContainer
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

#### Option2: Launch with ComponentContainer
If you are launching a node within `ComponentContainerCallbackIsolated`, all you have to do is modifying the launch file as below.
```xml
<launch>
  <node_container pkg="rclcpp_component_container_callback_isolated" exec="component_container_callback_isolated" name="sample_container" namespace="">
    <composable_node pkg="rclcpp_component_container_callback_isolated" plugin="SampleNode" name="sample_node" namespace="">
      ...
    </composable_node>
  </node_container>
</launch>
```

### Step2: Rebuild your app
If you modify the application's source code, a rebuild is necessary.

### Step3: Generate yaml template file
Open two terminal windows.
In one, launch a ROS 2 node to create a template for the YAML configuration file (`prerun` node), and in the other, launch the target ROS 2 application.
After launching the `prerun` node, launch your ROS 2 application.

To launch the `prerun` node, type the command below.
```bash
$ ros2 run ros2_thread_configurator thread_configurator_node --prerun
```

Then launch your ROS 2 application in another terminal window, after which you can see log messages like shown below in the `prerun` node window.
Each entry corresponds to the callback group ID and its OS thread ID.

![Screenshot from 2024-05-09 14-03-37](https://github.com/sykwer/callback_isolated_executor/assets/18254663/441528a5-b822-48b1-b334-4ca893041acc)

Once all the nodes of the target application are up and the logs in the prerun node window have stopped, press Control+C in the prerun node window.
Then, in the current directory, a template for the YAML configuration file `template.yaml` will be created in the format like below.

```yaml
callback_groups:
  - id: /sample_node@Subscription(/parameter_events)@Service(/sample_node/get_parameters)@Service(/sample_node/get_parameter_types)@Service(/sample_node/set_parameters)@Service(/sample_node/set_parameters_atomically)@Service(/sample_node/describe_parameters)@Service(/sample_node/list_parameters)@Waitable@Waitable@Waitable@Waitable
    affinity: ~
    policy: SCHED_OTHER
    priority: 0

  - id: /sample_node@Subscription(/topic_in)@Waitable
    affinity: ~
    policy: SCHED_OTHER
    priority: 0

  - id: /sample_node@Timer(1333000000)
    affinity: ~
    policy: SCHED_OTHER
    priority: 0

  - id: /sample_node@Timer(3000000000)
    affinity: ~
    policy: SCHED_OTHER
    priority: 0
```

Once the creation of template.yaml is complete, please also terminate the target ROS 2 application.

### Step4: Edit yaml file for scheduler configuration
Change the file name and edit to configure each callback group.
```bash
$ mv template.yaml your_config.yaml
$ vim your_config.yaml
```

For callback groups that do not require configuration, you can either delete the entry entirely or leave it as is because the default values in `template.yaml` are set with default nice values and no affinity settings on the CFS scheduler.
For the detailed specifications of the configuration file, please refer to https://github.com/sykwer/callback_isolated_executor/tree/main/ros2_thread_configurator#yaml-configuration-file-format.

### Step5: Launch your app with scheduler configuration
To launch the target ROS 2 application with the scheduler settings applied from the your_config.yaml you created, first start the configurator node with the following command.

```bash
$ ros2 run ros2_thread_configurator thread_configurator_node --config-file your_config.yaml
```

If there is a callback group with the `SCHED_DEADLINE` scheduling policy specified, running the configurator node requires root privileges.
This is because it is not possible to set threads to SCHED_DEADLINE within the permissions that can be granted through capability.
Note that if the target ROS 2 application is operating with a specific ROS_DOMAIN_ID, the configurator node must also be operated with the same ROS_DOMAIN_ID.

```bash
$ sudo bash -c "export ROS_DOMAIN_ID=[app domain id]; source /path/to/callback_isolated_executor/install/setup.bash; ros2 run ros2_thread_configurator thread_configurator_node --config-file your_config.yaml"
```

Immediately after launching the configurator node, it will print the settings and then wait for the target ROS 2 application to start running as follows.

![Screenshot from 2024-05-10 10-07-24](https://github.com/sykwer/callback_isolated_executor/assets/18254663/05b61107-fb2b-434b-befc-2cd98bffee1c)

In this state, when you launch the target ROS 2 application, the configurator node's window will display the message `Apply sched deadline?` and wait as below.
The entries above the waiting message each show the callback group ID and OS thread ID information received from the ROS 2 application.

![Screenshot from 2024-05-10 10-37-33](https://github.com/sykwer/callback_isolated_executor/assets/18254663/16dac6b4-acf0-4e00-bca2-096a10cbd5d1)

At this stage, settings with policies other than `SCHED_DEADLINE` have already been applied, while the application of settings including the `SCHED_DEADLINE` policy is postponed.
To apply settings that include the `SCHED_DEADLINE` policy, press the enter key in the window where `Apply sched deadline?` is displayed.

<details>
<summary>Why delayed configuration of the SCHED_DEADLINE policy?</summary>
  
We delay the applying of settings with the `SCHED_DEADLINE` policy because Autoware (main application area for this tool) contains nodes that implicitly create new threads immediately after startup, such as the EKF Localizer.
Threads specified with the `SCHED_DEADLINE` policy are prohibited from creating new child tasks.
Generally, real-time scheduling policies fail with an `EAGAIN` error when `clone(2)` is issued without the `SCHED_FLAG_RESET_ON_FORK` flag set.
However, setting this flag for `SCHED_DEADLINE` threads is impossible due to the following facts:
- According to [the Linux documentation for sched_setattr(2)](https://man7.org/linux/man-pages/man2/sched_setattr.2.html), the `flags` argument is currently required to be set to `0`, indicating that `SCHED_FLAG_RESET_ON_FORK` can only be set via `sched_setscheduler(2)`.
- According to [the Linux documentation for sched_setscheduler(2)](https://man7.org/linux/man-pages/man2/sched_setscheduler.2.html), the `SCHED_DEADLINE` policy can only be set via `sched_setattr(2)` and not through `sched_setscheduler(2)`.

Thus, we adopt a workaround where we delay only the settings that include the `SCHED_DEADLINE` policy.
Autoware's EKF Localizer creates child threads immediately after starting and does not create more afterwards (these child threads are likely deleted soon after).
Therefore, applying the `SCHED_DEADLINE` policy after the system has started, such as after starting playback of a rosbag or the operation of a real vehicle system, will not cause issues.
You need to apply the `SCHED_DEADLINE` policy only after the system has fully started up and no further creation of new child threads is expected.
</details>

![Screenshot from 2024-05-10 10-52-47](https://github.com/sykwer/callback_isolated_executor/assets/18254663/aad2151d-1ff8-496c-9fdf-c4ada7e22a76)

While the target ROS 2 application is running, the configurator node's window should not be closed and must remain open.
After the execution of the target ROS 2 application has ended, press the enter key in the window displaying `Press enter to exit and remove cgroups...`.
This will terminate the execution of the configurator node and simultaneously delete the cgroup that was created for setting the affinity of tasks with the `SCHED_DEADLINE` policy.


## TODO
### Why delayed configuration of SCHED_DEADLINE policy
WIP

### Why named `StaticCallbackIsolatedExecutor`
WIP
