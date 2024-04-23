cmake_minimum_required(VERSION 3.8)
project(rclcpp_component_container_callback_isolated)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

find_package(thread_config_msgs REQUIRED)
find_package(ros2_thread_configurator REQUIRED)
find_package(thread_config_msgs REQUIRED)

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

add_library(sample_node SHARED src/sample_node.cpp src/static_callback_isolated_executor.cpp)
target_compile_definitions(sample_node PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(sample_node rclcpp rclcpp_components std_msgs ros2_thread_configurator)
target_include_directories(sample_node PRIVATE
  ${ros2_thread_configurator_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
rclcpp_components_register_nodes(sample_node "SampleNode")

add_executable(sample_node2 src/sample_node.cpp src/static_callback_isolated_executor.cpp)
ament_target_dependencies(sample_node2 rclcpp rclcpp_components std_msgs ros2_thread_configurator)
target_include_directories(sample_node2 PRIVATE
  ${ros2_thread_configurator_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(component_container_callback_isolated src/component_container_callback_isolated.cpp)
ament_target_dependencies(component_container_callback_isolated rclcpp rclcpp_components ros2_thread_configurator)

add_executable(component_container_single src/component_container_single.cpp)
ament_target_dependencies(component_container_single rclcpp rclcpp_components ros2_thread_configurator)

add_library(static_callback_isolated_executor SHARED src/static_callback_isolated_executor.cpp)
ament_target_dependencies(static_callback_isolated_executor rclcpp ros2_thread_configurator)
target_include_directories(static_callback_isolated_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS static_callback_isolated_executor
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS sample_node2 component_container_callback_isolated component_container_single DESTINATION lib/${PROJECT_NAME})
install(TARGETS sample_node DESTINATION lib)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_export_dependencies(ros2_thread_configurator thread_config_msgs)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
