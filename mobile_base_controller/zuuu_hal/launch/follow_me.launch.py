from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():

    rplidar_launch_dir = os.path.join(
        get_package_share_directory('rplidar_ros2'), 'launch')
    print("TEST: {}".format(rplidar_launch_dir))
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rplidar_launch_dir, 'view_rplidar_s2_launch.py')),
        ),
        Node(
            package='zuuu_hal',
            namespace='follow_me',
            executable='follow_me',
            name='follow_me'
        ),

    ])
