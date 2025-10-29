from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_ros2',
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[
                # 可在此添加参数，例如 serial_port, baudrate 等
                # {'serial_port': '/dev/ttyUSB0'},
                # {'baudrate': 115200},
            ]
        )
    ])
