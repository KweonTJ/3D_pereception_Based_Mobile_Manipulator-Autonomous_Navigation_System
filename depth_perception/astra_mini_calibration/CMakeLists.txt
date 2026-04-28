cmake_minimum_required(VERSION 3.8)
project(astra_mini_calibration)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/cameracalibrator_with_save.py
  scripts/camera_info_from_json.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
