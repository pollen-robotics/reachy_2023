from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controllers_file_arg = DeclareLaunchArgument(
        'controllers_file',
        default_value=['reachy_no_orbita_controllers.yaml'],
        description='YAML file with the controllers configuration.',
    )
    controllers_file = LaunchConfiguration('controllers_file')

    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Start RViz2 automatically with this launch file.',
    )
    start_rviz = LaunchConfiguration('start_rviz')

    arguments = [
        controllers_file_arg,
        start_rviz_arg,
    ]

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('reachy_description'), 'urdf', 'reachy.urdf.xacro']
            ),
            ' ',
        ]
    )
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
    }

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('reachy_bringup'),
            'config',
            controllers_file,
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('reachy_description'), 'config', 'reachy.rviz']
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='screen',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(start_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    neck_forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['neck_forward_position_controller', '-c', '/controller_manager'],
    )

    r_arm_forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['r_arm_forward_position_controller', '-c', '/controller_manager'],
    )

    l_arm_forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['l_arm_forward_position_controller', '-c', '/controller_manager'],
    )

    antenna_forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['antenna_forward_position_controller', '-c', '/controller_manager'],
    )

    gripper_forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_forward_position_controller', '-c', '/controller_manager'],
    )

    forward_torque_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_torque_controller', '-c', '/controller_manager'],
    )

    pid_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['pid_controller', '-c', '/controller_manager'],
    )

    forward_fan_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_fan_controller', '-c', '/controller_manager'],
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                # neck_forward_position_controller_spawner,
                r_arm_forward_position_controller_spawner,
                l_arm_forward_position_controller_spawner,
                antenna_forward_position_controller_spawner,
                gripper_forward_position_controller_spawner,
                forward_torque_controller_spawner,
                pid_controller_spawner,
                forward_fan_controller_spawner,
            ],
        ),
    )

    return LaunchDescription(arguments + [
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ])
