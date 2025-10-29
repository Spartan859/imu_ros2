from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_ros2',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),
        Node(
            package='imu_ros2',
            executable='imu_serial_node.py',
            name='imu_serial_node',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB0'},
                {'baud': 115200}
            ]
        )
    ])
