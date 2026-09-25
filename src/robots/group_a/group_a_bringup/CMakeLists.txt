cmake_minimum_required(VERSION 3.8)
project(group_a_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS scripts/gripper_manager.py scripts/force_sensor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
