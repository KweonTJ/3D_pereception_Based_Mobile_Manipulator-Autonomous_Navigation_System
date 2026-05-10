cmake_minimum_required(VERSION 3.8)
project(leader_platooning_beacon)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)

add_executable(leader_platooning_beacon_node src/leader_platooning_beacon_node.cpp)
ament_target_dependencies(
  leader_platooning_beacon_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
)

add_executable(
  battery_state_from_dynamic_joint_state_node
  src/battery_state_from_dynamic_joint_state_node.cpp
)
ament_target_dependencies(
  battery_state_from_dynamic_joint_state_node
  rclcpp
  sensor_msgs
  control_msgs
)

install(TARGETS
  leader_platooning_beacon_node
  battery_state_from_dynamic_joint_state_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
