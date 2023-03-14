import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('zuuu_hal'),
        'config',
        'params.yaml'
    )

    rplidar_launch_dir = os.path.join(
        get_package_share_directory('rplidar_ros2'), 'launch')

    # Launch arguments
    arguments = [
        DeclareLaunchArgument(name='use_sim_time', default_value='False',
                              description='Flag to enable use_sim_time'),
    ]

    launches = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rplidar_launch_dir, 'zuuu_rplidar_s2_launch.py')),
        ),
    ]

    nodes = [
        Node(
            package='zuuu_hal',
            name='zuuu_hal',
            executable='hal',
            parameters=[config]
        )
    ]

    return LaunchDescription(arguments + launches + nodes)
