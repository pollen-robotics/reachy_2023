from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode
from reachy_bringup.launch_shared import fake_launch_arg, gazebo_launch_arg


def generate_launch_description():
    fake = LaunchConfiguration(fake_launch_arg.name)
    gazebo = LaunchConfiguration(gazebo_launch_arg.name)

    return LaunchDescription([
        fake_launch_arg,
        gazebo_launch_arg,
        LifecycleNode(
            name='fake_camera',
            namespace='',
            package='reachy_fake',
            executable='fake_camera',
            condition=IfCondition(
                PythonExpression(["\"", fake, "\" == \"true\""]),
                # would be better if arg was True/False either than true/false
                # keep it that way for overall consistency with other roslaunch args
            )),
        LifecycleNode(
            name='fake_zoom',
            namespace='',
            package='reachy_fake',
            executable='fake_zoom',
            condition=IfCondition(
                PythonExpression(["\"", fake, "\" == \"true\" or \"", gazebo, "\" == \"true\""])
            )
        )
    ])
