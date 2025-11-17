from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    imu_port_arg = LaunchConfiguration('port', default='/dev/ttyUSB1')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB1',
            description='IMU serial port path (e.g., /dev/ttyUSB1 or COM5)'
        ),
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
                {'port': imu_port_arg},
                {'baud': 115200}
            ]
        )
    ])
