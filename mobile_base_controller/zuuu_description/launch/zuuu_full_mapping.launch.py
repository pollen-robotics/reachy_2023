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
    # Path finding (peak comedy)
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package='zuuu_description').find('zuuu_description')
    default_rviz_config_path = os.path.join(
        pkg_share, 'rviz/mapping.rviz')
    # rviz_config_dir = os.path.join(
    #     get_package_share_directory('rplidar_ros2'),
    #     'rviz',
    #     'rplidar_ros2.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Launch arguments
    arguments = [
        DeclareLaunchArgument(name='use_sim_time', default_value='False',
                              description='Flag to enable use_sim_time'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                              description='Absolute path to rviz config file'),
    ]

    nodes = [

    ]
    # launch_arguments={'use_sim_time': False}.items(),

    # Launch files to call
    launches = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch',
                             'zuuu_bringup.launch.py')
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch',
                             'mapping.launch.py')
            ),
        ),

    ]

    return LaunchDescription(arguments + launches + nodes)
