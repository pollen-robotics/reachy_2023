import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    zuuu_hal_dir = get_package_share_directory('zuuu_hal')

    arguments = []

    launches = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(zuuu_hal_dir, 'hal.launch.py')
            )
        )
    ]

    nodes = [
        Node(
            package='mobile_base_sdk_server',
            executable='mobile_base_sdk_server',
        ),
    ]

    return LaunchDescription(arguments + launches + nodes)
