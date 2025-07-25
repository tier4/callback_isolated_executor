cmake_minimum_required(VERSION 3.8)
project(callback_isolated_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

find_package(cie_config_msgs REQUIRED)
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

add_executable(component_container_callback_isolated src/component_container_callback_isolated.cpp)
ament_target_dependencies(component_container_callback_isolated rclcpp rclcpp_components cie_thread_configurator)

add_executable(component_container_single src/component_container_single.cpp)
ament_target_dependencies(component_container_single rclcpp rclcpp_components cie_thread_configurator)

add_library(callback_isolated_executor SHARED src/callback_isolated_executor.cpp)
ament_target_dependencies(callback_isolated_executor rclcpp cie_thread_configurator)
target_include_directories(callback_isolated_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS callback_isolated_executor
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

install(
  TARGETS component_container_callback_isolated component_container_single
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(cie_thread_configurator cie_config_msgs)

ament_package()
