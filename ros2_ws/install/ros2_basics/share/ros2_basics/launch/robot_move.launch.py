from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_basics',
            namespace='',
            executable='move_robot',
            name='move_robot',
        )
    ])