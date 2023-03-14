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

    # Launch arguments
    arguments = [
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                              description='Flag to enable use_sim_time'),
    ]

    # Launch files to call
    launches = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch',
                             'gazebo_simulation.launch.py')
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch',
                             'navigation.launch.py')
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch',
                             'rviz_navigation.launch.py')
            ),
        ),
    ]

    return LaunchDescription(arguments + launches)
