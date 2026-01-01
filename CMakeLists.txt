cmake_minimum_required(VERSION 3.8)
project(assignment2_rt)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleInfo.msg"
  "srv/GetAverages.srv"
  "srv/SetThreshold.srv"
)
ament_export_dependencies(rosidl_default_runtime)

install(PROGRAMS
  scripts/user_controller
  scripts/safety_detector
  scripts/rollback_controller
  scripts/cmd_mux
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
