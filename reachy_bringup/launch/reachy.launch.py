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

from reachy_bringup.launch_shared import FULL_KIT, STARTER_KIT_RIGHT, STARTER_KIT_LEFT, get_reachy_config, \
    fake_launch_arg, gazebo_launch_arg, robot_model_launch_arg, get_robot_controllers, get_robot_description


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
    robot_model_rl = LaunchConfiguration('robot_model')
    robot_model_py = robot_model_rl.perform(context)

    # Robot model
    robot_model_file = get_reachy_config()
    if robot_model_file:
        LogInfo(msg="Using robot_model described in ~/.reachy.yaml ...").execute(context=context)
        robot_model_py = robot_model_file
    LogInfo(msg="Robot Model :: {}".format(robot_model_py)).execute(context=context)
    robot_controllers = get_robot_controllers(robot_model_py)

    robot_description = get_robot_description(robot_model_py, fake_py, gazebo_py)

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='screen',
        condition=IfCondition(
            PythonExpression(f"not {gazebo_py}")
        )  # For Gazebo simulation, we should not launch the controller manager (Gazebo does its own stuff)
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

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare("reachy_bringup"), '/launch', '/rviz.launch.py']),
                    condition=IfCondition(start_rviz_rl)
                )],
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
        launch_arguments={'robot_config': f'{robot_model_py}'}.items(),
        condition=IfCondition(gazebo_rl)
    )

    controller_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("reachy_bringup"), '/launch', '/reachy_controllers.launch.py']),
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

    delay_sdk_server_after_kinematics = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=kinematics_node, goal_state='inactive',
            entities=[
                LogInfo(msg='Kinematics started started, spawning sdk', condition=IfCondition(start_sdk_server_rl)),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare("reachy_sdk_server"), '/reachy_camera_sdk_server.launch.py']),
                    condition=IfCondition(start_sdk_server_rl),
                )],
        )
    )

    fake_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("reachy_fake"), '/launch/reachy_fake.launch.py']),
    )

    return [
        control_node,
        gazebo_node,
        fake_nodes,
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
            'start_sdk_server',
            default_value='false',
            description='Start sdk_server along with reachy nodes with this launch file.',
            choices=['true', 'false']
        ),
        fake_launch_arg,
        gazebo_launch_arg,
        robot_model_launch_arg,
        OpaqueFunction(function=launch_setup)
    ])

# TODO use a OnProcessIO to check whether every node has sent its 'OK' message and log accordingly ?
