cmake_minimum_required(VERSION 3.8)
project(custom_msgs)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 1. Compile your service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SpawnBox.srv"
)

ament_package()
