cmake_minimum_required(VERSION 3.5)
project(imu_ros2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ImuData.msg"
  DEPENDENCIES std_msgs
)

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

# Link generated messages to executable
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(imu_node ${cpp_typesupport_target})

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

# Export dependencies for downstream packages
ament_export_dependencies(rosidl_default_runtime)

ament_package()