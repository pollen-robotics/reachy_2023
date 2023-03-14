import launch
import launch_ros
import os

from launch import LaunchDescription
from launch import event_handlers
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package='zuuu_description').find('zuuu_description')
    default_rviz_config_path = os.path.join(
        pkg_share, 'rviz/lidar.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_sim_time_param = {
        'use_sim_time': use_sim_time}

    # Launch arguments
    arguments = [
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                              description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                              description='Flag to enable use_sim_time'),
    ]

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[use_sim_time_param],
    )
    nodes = [rviz_node]

    return LaunchDescription(arguments+nodes)
