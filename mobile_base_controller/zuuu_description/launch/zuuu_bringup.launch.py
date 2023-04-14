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
        pkg_share, 'rviz/bringup.rviz')
    zuuu_hal_launch_dir = get_package_share_directory('zuuu_hal')

    # rviz_config_dir = os.path.join(
    #     get_package_share_directory('rplidar_ros2'),
    #     'rviz',
    #     'rplidar_ros2.rviz')

    rplidar_launch_dir = os.path.join(
        get_package_share_directory('rplidar_ros2'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_sim_time_param = {
        'use_sim_time': use_sim_time}

    # Launch arguments
    arguments = [
        DeclareLaunchArgument(name='use_sim_time', default_value='False',
                              description='Flag to enable use_sim_time'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                              description='Absolute path to rviz config file'),
    ]

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen',
        parameters=[use_sim_time_param],)

    nodes = [
        rviz,

    ]

    # Launch files to call
    launches = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch',
                             'description_bringup.launch.py')
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(zuuu_hal_launch_dir, 'hal.launch.py')),
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(pkg_share, 'launch',
        #                      'navigation.launch.py')
        #     ),
        # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(pkg_share, 'launch',
        #                      'rviz_navigation.launch.py')
        #     ),
        # ),
    ]

    return LaunchDescription(arguments + launches + nodes)
