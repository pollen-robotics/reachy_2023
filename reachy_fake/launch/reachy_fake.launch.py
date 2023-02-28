from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    fake = LaunchConfiguration('fake')
    gazebo = LaunchConfiguration('gazebo')

    return LaunchDescription([
        DeclareLaunchArgument(
            'fake',
            default_value='false',
            description='Start on fake_reachy mode with this launch file.',
            choices=['true', 'false']
        ),
        DeclareLaunchArgument(
            'gazebo',
            default_value='false',
            description='Start a fake_hardware with gazebo as simulation tool.',
            choices=['true', 'false']
        ),
        Node(
            package='reachy_fake',
            executable='fake_camera',
            condition=IfCondition(
                PythonExpression(["\"", fake, "\" == \"true\""]),
                # would be nicer if arg was True/False either than true/false
                # keep it that way for overall consistency with other roslaunch args
            )),
        Node(
            package='reachy_fake',
            executable='fake_zoom',
            condition=IfCondition(
                PythonExpression(["\"", fake, "\" == \"true\" or \"", gazebo, "\" == \"true\""])
            ),
        )
    ])
