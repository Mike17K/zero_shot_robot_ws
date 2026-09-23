cmake_minimum_required(VERSION 3.8)
project(custom_msgs)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CartesianMove.action"
  "action/NavigateToNode.action"
  "msg/BoxFactoryStatus.msg"
  "msg/GripperState.msg"
  "msg/InfeedLineState.msg"
  "msg/SpawnedBox.msg"
  "srv/SpawnBox.srv"
  DEPENDENCIES geometry_msgs
)

ament_package()
