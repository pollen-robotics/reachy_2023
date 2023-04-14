from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='reachy_sdk_server',
            executable='reachy_sdk_server',
        ),
    ])