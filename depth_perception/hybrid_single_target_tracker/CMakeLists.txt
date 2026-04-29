cmake_minimum_required(VERSION 3.8)
project(hybrid_single_target_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(HYBRID_SINGLE_TARGET_TRACKER_FORCE_CAMSHIFT "Build with the CamShift fallback even when OpenCV tracking is available" OFF)
option(HYBRID_SINGLE_TARGET_TRACKER_PREFER_ROS_OPENCV "Prefer the Ubuntu/ROS OpenCV package over custom OpenCV builds" ON)

set(opencv_optional_components tracking)
if(HYBRID_SINGLE_TARGET_TRACKER_FORCE_CAMSHIFT)
  set(opencv_optional_components)
endif()

if(HYBRID_SINGLE_TARGET_TRACKER_PREFER_ROS_OPENCV)
  foreach(opencv_config_dir
    "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/opencv4"
    "/usr/lib/x86_64-linux-gnu/cmake/opencv4"
  )
    if(EXISTS "${opencv_config_dir}/OpenCVConfig.cmake")
      set(OpenCV_DIR "${opencv_config_dir}" CACHE PATH "OpenCV config directory" FORCE)
      break()
    endif()
  endforeach()
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc video OPTIONAL_COMPONENTS ${opencv_optional_components})
message(STATUS "hybrid_single_target_tracker using OpenCV ${OpenCV_VERSION} from ${OpenCV_DIR}")

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

if(OpenCV_tracking_FOUND AND NOT HYBRID_SINGLE_TARGET_TRACKER_FORCE_CAMSHIFT)
  target_compile_definitions(${PROJECT_NAME}_node PRIVATE HYBRID_SINGLE_TARGET_TRACKER_HAS_OPENCV_TRACKING=1)
elseif(HYBRID_SINGLE_TARGET_TRACKER_FORCE_CAMSHIFT)
  message(WARNING "Building hybrid_single_target_tracker with the forced CamShift fallback; CSRT is disabled for this build.")
else()
  message(WARNING "OpenCV tracking module was not found. Building hybrid_single_target_tracker with a CamShift fallback; install libopencv-contrib-dev for CSRT.")
endif()

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
