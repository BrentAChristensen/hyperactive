cmake_minimum_required(VERSION 3.5)
project(impact_rqt_serial_monitor)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED) # For Python nodes
find_package(std_msgs REQUIRED) # Example dependency, adjust as necessary

# Install Python scripts
install(
  PROGRAMS
    impact_rqt_serial_monitor/impact_rqt_serial_monitor/serial_monitor.py  # Add your scripts here
    # Add more scripts if needed
  DESTINATION lib/${PROJECT_NAME}
)

# Install the icons directory
install(DIRECTORY 
  resource/icons
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.png"  # Adjust this pattern as needed to match your icon files
)

# Optionally install configuration files, data, etc.
 install(FILES resource/icons/impact_rqt_serial_monitor.png
    DESTINATION share/${PROJECT_NAME}/resource/icons
 )

# Add tests (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
