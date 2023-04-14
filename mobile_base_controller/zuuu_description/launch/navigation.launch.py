import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

MAP_NAME = os.environ['ZUUU_MAP_NAME']


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_path = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('zuuu_description'),
            'maps',
            MAP_NAME + '.yaml'))

    param_file_name = 'nav2_params.yaml'
    param_path = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('zuuu_description'),
            'config/',
            param_file_name))

    bt_file_name = 'zuuu_bt.xml'
    bt_path = LaunchConfiguration(
        'bt_file',
        default=os.path.join(
            get_package_share_directory('zuuu_description'),
            'config/',
            bt_file_name))

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_path,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_path,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            name='default_bt_xml_filename',
            default_value=bt_path,
            description='Full path to the behavior tree xml file to use'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_path,
                'use_sim_time': use_sim_time,
                'params_file': os.path.join(
                    get_package_share_directory('zuuu_description'),
                    'config/',
                    param_file_name)}.items(),
        ) # TODO force argument to bypass current issue, investigation needed here
    ])
