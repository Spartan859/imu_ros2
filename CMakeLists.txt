cmake_minimum_required(VERSION 3.5)
project(imu_ros2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(include)

# Source files
set(SOURCES
  src/imu_node.cpp
  src/imu_parser.cpp
)

# Create executable
add_executable(imu_node ${SOURCES})

# Link libraries
ament_target_dependencies(imu_node
  rclcpp
  std_msgs
  sensor_msgs
)

# Install executable
install(TARGETS
  imu_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include
)


# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install message files
install(DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

# Install Python scripts
install(PROGRAMS scripts/imu_serial_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()