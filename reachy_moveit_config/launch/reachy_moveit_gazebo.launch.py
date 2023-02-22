from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command, FindExecutable

from launch_ros.actions import Node, SetUseSimTime
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from srdfdom.srdf import SRDF


from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

FULL_KIT, STARTER_KIT_RIGHT, STARTER_KIT_LEFT = 'full_kit', 'starter_kit_right', 'starter_kit_left'


def get_reachy_config():
    import yaml
    import os
    config_file = os.path.expanduser('~/.reachy.yaml')
    try:
        with open(config_file) as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            return config["model"] if config["model"] in [FULL_KIT, STARTER_KIT_RIGHT, STARTER_KIT_LEFT] else False
    except (FileNotFoundError, TypeError):
        return False


robot_model_file = get_reachy_config()


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
    SetUseSimTime(True)
    robot_model = "full_kit"
    if robot_model_file:
        # TODO find a ROS way to log (without rebuilding a whole node ?
        print("Using robot_model described in ~/.reachy.yaml ...")
        robot_model = robot_model_file
    print("Robot Model :: {}".format(robot_model))

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock.",
        ),
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

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

    gz_world = PathJoinSubstitution(
        [FindPackageShare('reachy_gazebo'), 'config', 'reachy_grasping.sdf'])

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("reachy_gazebo"), '/launch', '/gazebo.launch.py']),
            launch_arguments={'robot_config': f'{robot_model}', 'use_sim_time': f'{use_sim_time}', 'world': f'{gz_world}'}.items()
        )
    )

    # Given the published joint states, publish tf for the robot links
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/rsp.launch.py")
            ),
            launch_arguments={'use_sim_time': f'{use_sim_time}'}.items()
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/move_group.launch.py")
            ),
            launch_arguments={'use_sim_time': f'{use_sim_time}'}.items()
        )
    )

    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            launch_arguments={'use_sim_time': f'{use_sim_time}'}.items()
        )
    )

    # If database loading was enabled, start mongodb as well
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
            launch_arguments={'use_sim_time': f'{use_sim_time}'}.items()
        )
    )

    ld.add_action(

        TimerAction(
            period=5.0,
            actions=[

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(moveit_config.package_path / "launch/spawn_controllers.launch.py")
                    ),
                    launch_arguments={'use_sim_time': f'{use_sim_time}'}.items()
                )
            ],
        )
    )

    return ld


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("reachy_2023", package_name="reachy_moveit_config")
    moveit_config = moveit_config.sensors_3d(None)  # be sure to disable the 3D sensor

    # moveit_config = moveit_config.robot_description(mappings={'use_fake_hardware': 'true', 'use_gazebo': 'true', 'use_moveit_gazebo': 'true'})  # pass parameters to xacro (this should work be it does not...)

    return generate_demo_launch(moveit_config.to_moveit_configs())
