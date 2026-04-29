cmake_minimum_required(VERSION 3.8)
project(hybrid_csrt_ibvs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc tracking)

add_executable(csrt_ibvs_node
  src/csrt_ibvs_main.cpp
  src/csrt_ibvs_node.cpp
)

target_compile_features(csrt_ibvs_node PUBLIC cxx_std_17)
target_include_directories(csrt_ibvs_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(csrt_ibvs_node ${OpenCV_LIBS})
ament_target_dependencies(csrt_ibvs_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
)

install(TARGETS csrt_ibvs_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS tools/publish_bbox.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
