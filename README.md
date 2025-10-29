# imu_ros2

## Overview
The `imu_ros2` package is designed for parsing and handling Inertial Measurement Unit (IMU) data in a ROS2 environment. It provides functionalities to receive, process, and publish IMU data, including acceleration, gyroscope readings, and Euler angles.

## Features
- **IMU Data Parsing**: Efficiently parses raw IMU data using the `ImuParser` class.
- **Custom Message Type**: Defines a custom message type `ImuData` for transmitting IMU data.
- **Node Implementation**: Implements an IMU node that subscribes to and publishes IMU data.

## Installation
To build and install the `imu_ros2` package, follow these steps:

1. Clone the repository:
   ```
   git clone <repository_url>
   cd imu_ros2
   ```

2. Install dependencies:
   ```
   rosdep install -i --from-path src --rosdistro <ros_distro> -y
   ```

3. Build the package:
   ```
   colcon build
   ```

4. Source the setup file:
   ```
   source install/setup.bash
   ```

## Usage
To launch the IMU node, use the provided launch file:

```
ros2 launch imu_ros2 test_imu_launch.py
```

This will start the IMU node and begin processing incoming IMU data.

## File Structure
- `CMakeLists.txt`: CMake configuration file for building the project.
- `package.xml`: ROS2 package configuration file containing metadata and dependencies.
- `README.md`: Documentation for the `imu_ros2` package.
- `include/imu_ros2/imu_parser.hpp`: Header file defining the `ImuParser` class for IMU data parsing.
- `launch/test_imu_launch.py`: Launch file for starting the IMU node.
- `src/imu_node.cpp`: Implementation of the IMU node.
- `src/imu_parser.cpp`: Implementation of the `ImuParser` class.
- `msg/ImuData.msg`: Definition of the custom message type for IMU data.