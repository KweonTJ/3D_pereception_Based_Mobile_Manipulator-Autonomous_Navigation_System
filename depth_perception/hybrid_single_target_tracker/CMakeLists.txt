cmake_minimum_required(VERSION 3.8)
project(hybrid_single_target_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc tracking)

add_executable(${PROJECT_NAME}_node
  src/hybrid_single_target_tracker_node.cpp
)

target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME}_node
  PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  message_filters
)

target_include_directories(hybrid_single_target_tracker_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(hybrid_single_target_tracker_node
  ${OpenCV_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
