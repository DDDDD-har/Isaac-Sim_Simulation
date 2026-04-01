cmake_minimum_required(VERSION 3.8)
project(wheel_loader_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Add executable
add_executable(simple_vehicle_controller_fullstate_node
  src/simple_vehicle_controller_fullstate.cpp
)

ament_target_dependencies(simple_vehicle_controller_fullstate_node
  rclcpp
  sensor_msgs
)

add_executable(curve_path_lateral_pid_tracker_node
  src/curve_path_lateral_pid_tracker.cpp
)

ament_target_dependencies(curve_path_lateral_pid_tracker_node
  rclcpp
  sensor_msgs
  nav_msgs
)

install(TARGETS
  simple_vehicle_controller_fullstate_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  curve_path_lateral_pid_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# Install include files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Test dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
