cmake_minimum_required(VERSION 3.8)
project(conveyor_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/belt_speed_relay.py scripts/box_factory.py DESTINATION lib/${PROJECT_NAME})

ament_package()
