from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # See: https://github.com/ros2/rosidl_python/issues/79
        SetEnvironmentVariable('PYTHONOPTIMIZE', '1'),

        Node(
            package='camera_controllers',
            executable='camera_publisher',
        ),
        Node(
            package='camera_controllers',
            executable='camera_focus',
        ),
        Node(
            package='camera_controllers',
            executable='camera_zoom_service',
        ),
    ])