import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    use_sim_time_param = {
        'use_sim_time': use_sim_time}

    param_file_name = 'slam.yaml'
    param_dir = LaunchConfiguration(
        'parameters',
        default=os.path.join(
            get_package_share_directory('zuuu_description'),
            'config/',
            param_file_name))

    print("To save the map, go to the /maps directory and call the map saver like this:\nros2 run nav2_map_server map_saver_cli -f hospital -t /map")

    return LaunchDescription([
        DeclareLaunchArgument(
            'parameters',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            "log_level",
            default_value=["debug"],
            description="Logging level",
        ),

        Node(
            package='slam_toolbox', executable='sync_slam_toolbox_node', output='screen',
            name='slam_toolbox', parameters=[param_dir, use_sim_time_param])
    ])
