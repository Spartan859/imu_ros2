from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_ros2',
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[{
                'param_name': 'param_value'  # Replace with actual parameters if needed
            }],
            remappings=[
                # Add any topic remappings if necessary
            ]
        ),
    ])