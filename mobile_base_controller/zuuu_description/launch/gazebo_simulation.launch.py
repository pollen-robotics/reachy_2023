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
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    default_model_path = os.path.join(
        pkg_share, 'urdf/zuuu.urdf.xacro')

    # Set the path to the world file
    # 'hospital.world'  # 'my_world.sdf'
    world_file_name = 'hospital.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)

    # Paths to the robot description and its controllers
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('zuuu_description'),
                 'urdf', 'zuuu.urdf.xacro']
            ),
            ' ',
            'use_gazebo:=true',
            ' ',
            'use_fake_components:=false',
            ' ',
            'use_fixed_wheels:=true',
            ' ',
            'use_ros_control:=true',
        ]
    )
    robot_description = {
        "robot_description": robot_description_content}
    zuuu_controller = PathJoinSubstitution(
        [
            FindPackageShare(
                "zuuu_description"),
            "config",
            "zuuu_controllers.yaml",
        ]
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_sim_time_param = {
        'use_sim_time': use_sim_time}

    # Launch arguments
    arguments = [
        DeclareLaunchArgument(
            'controllers_file',
            default_value=['zuuu_controllers.yaml'],
            description='YAML file with the controllers configuration.',
        ),
        DeclareLaunchArgument(name='gui', default_value='True',
                              description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                              description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                              description='Flag to enable use_sim_time'),
    ]

    # Nodes declaration
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, use_sim_time_param]
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, zuuu_controller, use_sim_time_param],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    # Call this to have 'ros2 control list_controllers' give :
    # joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[use_sim_time_param],
        output='screen',
    )
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[use_sim_time_param],
    #     condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    # )

    # To get all the available options:
    # ros2 run gazebo_ros spawn_entity.py -h
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'zuuu', '-topic',
                   'robot_description', '-timeout', '60',  '-x', '0', '-y', '2'],

        output='screen'
    )
    # Beware! The libgazebo_ros_planar_move.so plugin already published the topic /odom AND the odom TFs.
    # So the robot_localization_node should either not be called or at least never be configured to write
    # its output on /odom (cf. config/ekf.yaml).
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'),
                    use_sim_time_param]
    )

    # Nodes to call
    nodes = [
        # joint_state_publisher_node,
        joint_state_broadcaster_spawner,
        robot_state_publisher_node,
        spawn_entity,
        controller_manager_node,
        # robot_localization_node,
    ]

    # Launch files to call
    launches = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_path, 'verbose': 'true'}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),
    ]

    return LaunchDescription(arguments + launches + nodes)


""" 
Infamous bug where the TFs from map to anything other than odom was outdated (with the weird 0.2 timestamp) 
solved by calling libgazebo_ros_init.so and libgazebo_ros_factory.so

https://discourse.ros.org/t/spawning-a-robot-entity-using-a-node-with-gazebo-and-ros-2/9985
"""
