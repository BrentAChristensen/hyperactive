cmake_minimum_required(VERSION 3.5)
project(feather_tracker)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)

install(DIRECTORY launch scripts
  DESTINATION share/${PROJECT_NAME}
)

install (PROGRAMS 
  scripts/follow_feather.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()