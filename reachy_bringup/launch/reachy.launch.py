from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, TimerAction, \
    OpaqueFunction, LogInfo
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart, OnExecutionComplete
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node, SetUseSimTime, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from reachy_bringup.launch_shared import FULL_KIT, STARTER_KIT_RIGHT, STARTER_KIT_LEFT


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


def launch_setup(context, *args, **kwargs):
    # perform(context) returns arg as a string, hence the conversion
    # var_rl is a ROS launch type object
    # var_py is a converted version, python friendly
    start_rviz_rl = LaunchConfiguration('start_rviz')
    start_rviz_py = start_rviz_rl.perform(context) == 'true'
    fake_rl = LaunchConfiguration('fake')
    fake_py = fake_rl.perform(context) == 'true'
    gazebo_rl = LaunchConfiguration('gazebo')
    gazebo_py = gazebo_rl.perform(context) == 'true'
    start_sdk_server_rl = LaunchConfiguration('start_sdk_server')
    start_sdk_server_py = start_sdk_server_rl.perform(context) == 'true'

    # Robot model
    robot_model_rl = LaunchConfiguration('robot_model')
    robot_model_py = robot_model_rl.perform(context)
    if robot_model_file:
        LogInfo(msg="Using robot_model described in ~/.reachy.yaml ...").execute(context=context)
        robot_model_py = robot_model_file
    LogInfo(msg="Robot Model :: {}".format(robot_model_py)).execute(context=context)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('reachy_description'), 'urdf', 'reachy.urdf.xacro']
            ),
            *((' ', 'use_fake_hardware:=true', ' ') if fake_py else
              (' ', 'use_fake_hardware:=true use_gazebo:=true depth_camera:=false', ' ') if gazebo_py else
              (' ',)),
            f'robot_config:={robot_model_py}',
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
            f'reachy_{robot_model_py}_controllers.yaml',
        ]
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

    gazebo_state_broadcaster_params = PathJoinSubstitution(
        [FindPackageShare('reachy_gazebo'), 'config', 'gz_state_broadcaster_params.yaml']
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[*(('joint_state_broadcaster', '-p', gazebo_state_broadcaster_params) if gazebo_py else
                     ('joint_state_broadcaster',)),
                   '--controller-manager',
                   '/controller_manager'],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('reachy_description'), 'config', 'reachy.rviz'])],
        condition=IfCondition(start_rviz_rl),
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
    )

    kinematics_node = LifecycleNode(
        name='kinematics',
        namespace='',
        package='reachy_kdl_kinematics',
        executable='reachy_kdl_kinematics',
    )

    dynamic_state_router_node = Node(
        package='dynamic_state_router',
        executable='dynamic_state_router',
        arguments=[robot_controllers],
    )

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("reachy_gazebo"), '/launch', '/gazebo.launch.py']),
        launch_arguments={'robot_config': f'{robot_model_py}'}.items()
    )

    controller_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("reachy_controllers"), '/launch', '/reachy_controllers.launch.py']),
        launch_arguments={'robot_model': f'{robot_model_py}',
                          'robot_controllers': robot_controllers}.items()
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                controller_nodes,
                kinematics_node
            ],
        ),
    )

    sdk_server_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("reachy_sdk_server"), '/reachy_camera_sdk_server.launch.py']),
        condition=IfCondition(start_sdk_server_rl),
    )

    delay_sdk_server_after_kinematics = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=kinematics_node, goal_state='inactive',
            entities=[sdk_server_nodes],
        )
    )

    return [
        # For Gazebo simulation, we should not launch the controller manager (Gazebo does its own stuff)
        *((control_node,) if not gazebo_py else
          (SetUseSimTime(True),  # does not seem to work...
           gazebo_node)),
        fake_camera_node,
        fake_zoom_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_sdk_server_after_kinematics,
        dynamic_state_router_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Start RViz2 automatically with this launch file.',
            choices=['true', 'false']
        ),
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
        DeclareLaunchArgument(
            'start_sdk_server',
            default_value='false',
            description='Start sdk_server along with reachy nodes with this launch file.',
            choices=['true', 'false']
        ),
        DeclareLaunchArgument(
            'robot_model',
            default_value=FULL_KIT,
            description='Choose robot_model configuration. '
                        'If a robot_configuration is defined in ~/.reachy.yaml : it WILL BE CHOSEN over any given arg',
            choices=[FULL_KIT, STARTER_KIT_RIGHT, STARTER_KIT_LEFT]
        ),
        OpaqueFunction(function=launch_setup)
    ])

# TODO use a OnProcessIO to check whether every node has sent its 'OK' message and log accordingly ?
