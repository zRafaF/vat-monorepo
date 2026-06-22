cmake_minimum_required(VERSION 3.8)
project(unitree_go)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TimeSpec.msg"
  "msg/IMUState.msg"
  "msg/SportModeState.msg"
)

ament_package()
