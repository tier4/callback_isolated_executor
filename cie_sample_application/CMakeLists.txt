cmake_minimum_required(VERSION 3.8)
project(cie_sample_application)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

find_package(callback_isolated_executor REQUIRED)
find_package(cie_thread_configurator REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

add_library(sample_node SHARED src/sample_node.cpp)
ament_target_dependencies(sample_node rclcpp rclcpp_components std_msgs cie_thread_configurator)
rclcpp_components_register_nodes(sample_node "SampleNode")

add_executable(sample_node_main src/sample_node_main.cpp)
target_link_libraries(sample_node_main sample_node)
ament_target_dependencies(sample_node_main rclcpp callback_isolated_executor)

add_executable(sample_non_ros_process src/sample_non_ros_process.cpp)
ament_target_dependencies(sample_non_ros_process cie_thread_configurator)

install(TARGETS
  sample_node_main
  sample_non_ros_process
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS sample_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_export_include_directories(include)
ament_export_libraries(sample_node)

ament_package()
