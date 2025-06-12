cmake_minimum_required(VERSION 3.5)
project(feather_tracker)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch config resource scripts
  DESTINATION share/${PROJECT_NAME}
)

install (PROGRAMS 
  feather_tracker/detection_to_pose_node.py
  feather_tracker/follow_feather.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()