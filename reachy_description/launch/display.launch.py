from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_package_arg = DeclareLaunchArgument(
        'description_package',
        default_value='reachy_description',
        description='Description package with robot URDF/xacro files. Usually the argument is not set, it enables use of a custom description.'
    )
    description_file_arg = DeclareLaunchArgument(
        'description_file',
        default_value='reachy.urdf.xacro',
        description='URDF/xacro description file with the robot.',
    )

    arguments = [
        description_package_arg,
        description_file_arg,
    ]
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(description_package),
                 'urdf', description_file]
            ),
            ' ',
            'robot_config:=starter_kit_left',
            '',
        ]
    )
    robot_description = {'robot_description': ParameterValue(
        robot_description_content, value_type=str)}

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), 'config', 'reachy.rviz']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription(arguments + [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ])
