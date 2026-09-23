cmake_minimum_required(VERSION 3.8)
project(box_factory_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_package()
