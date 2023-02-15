from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from srdfdom.srdf import SRDF


from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_demo_launch(moveit_config):
    """
    Launches a self contained demo

    Includes
     * static_virtual_joint_tfs
     * robot_state_publisher
     * move_group
     * moveit_rviz
     * warehouse_db (optional)
     * ros2_control_node + controller spawners
    """
    ld = LaunchDescription()
    ld.add_action(
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = (
        moveit_config.package_path / "launch/static_virtual_joint_tfs.launch.py"
    )
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # Given the published joint states, publish tf for the robot links
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/rsp.launch.py")
            ),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/move_group.launch.py")
            ),
        )
    )

    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # If database loading was enabled, start mongodb as well
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    # Fake joint driver
    ld.add_action(

        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="ros2_control_node",
                    parameters=[
                        moveit_config.robot_description,
                        str(moveit_config.package_path / "config/ros2_controllers.yaml"),
                    ],
                    output='both',
                    # arguments=['--ros-args', '--log-level', 'debug']
                )
            ],
        )
    )

    ld.add_action(

        TimerAction(
            period=3.0,
            actions=[

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(moveit_config.package_path / "launch/spawn_controllers.launch.py")
                    ),
                )
            ],
        )
    )

    return ld


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("reachy_2023", package_name="reachy_moveit_config")
    moveit_config = moveit_config.sensors_3d(None)  # be sure to disable the 3D sensor
    moveit_config = moveit_config.robot_description(mappings={'use_fake_hardware': 'true', 'use_gazebo': 'false', 'use_moveit_gazebo': 'false', 'depth_camera': 'false'})  # pass parameters to xacro

    return generate_demo_launch(moveit_config.to_moveit_configs())
