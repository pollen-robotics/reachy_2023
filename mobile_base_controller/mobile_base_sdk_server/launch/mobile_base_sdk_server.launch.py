from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mobile_base_sdk_server',
            executable='mobile_base_sdk_server',
        ),
    ])
