cmake_minimum_required(VERSION 3.8)
project(pinky_office_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  map
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
