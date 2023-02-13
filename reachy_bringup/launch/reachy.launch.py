from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, TimerAction, \
    OpaqueFunction, LogInfo
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node, SetUseSimTime
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

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


def launch_setup(context, *args, **kwargs):
    # perform(context) returns arg as a string, hence the conversion
    # var_arg is a ROS launch type object
    # var is a converted version, python friendly
    start_rviz_arg = LaunchConfiguration('start_rviz')
    start_rviz = start_rviz_arg.perform(context) == 'true'
    fake_arg = LaunchConfiguration('fake')
    fake = fake_arg.perform(context) == 'true'
    gazebo_arg = LaunchConfiguration('gazebo')
    gazebo = gazebo_arg.perform(context) == 'true'
    start_sdk_server_arg = LaunchConfiguration('start_sdk_server')
    start_sdk_server = start_sdk_server_arg.perform(context) == 'true'

    # Robot model
    robot_model_arg = LaunchConfiguration('robot_model')
    robot_model = robot_model_arg.perform(context)
    if robot_model_file:
        LogInfo(msg="Using robot_model described in ~/.reachy.yaml ...").execute(context=context)
        robot_model = robot_model_file
    LogInfo(msg="Robot Model :: {}".format(robot_model)).execute(context=context)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('reachy_description'), 'urdf', 'reachy.urdf.xacro']
            ),
            *((' ', 'use_fake_hardware:=true', ' ') if fake else
              (' ', 'use_fake_hardware:=true use_gazebo:=true depth_camera:=false', ' ') if gazebo else
              (' ',)),
            f'robot_config:={robot_model}',
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
            f'reachy_{robot_model}_controllers.yaml',
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('reachy_description'), 'config', 'reachy.rviz']
    )

    control_spawner = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='screen',
    )

    sdk_server_spawner = Node(
        package='reachy_sdk_server',
        executable='reachy_sdk_server',
        output='both',
        arguments=[robot_model],
        condition=IfCondition(start_sdk_server_arg),
    )

    sdk_camera_server_spawner = Node(
        package='reachy_sdk_server',
        executable='camera_server',
        output='both',
        condition=IfCondition(start_sdk_server_arg), 
    )

    robot_state_publisher_spawner = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    rviz_spawner = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(start_rviz_arg),
    )

    gazebo_state_broadcaster_params = PathJoinSubstitution(
        [FindPackageShare('reachy_gazebo'), 'config', 'gz_state_broadcaster_params.yaml']
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[*(('joint_state_broadcaster', '-p', gazebo_state_broadcaster_params) if gazebo else
                     ('joint_state_broadcaster',)),
                   '--controller-manager',
                   '/controller_manager'],
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
        condition=IfCondition(
            PythonExpression(f"'{robot_model}' == '{FULL_KIT}' or '{robot_model}' == '{STARTER_KIT_RIGHT}'")
        )
    )

    l_arm_forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['l_arm_forward_position_controller', '-c', '/controller_manager'],
        condition=IfCondition(
            PythonExpression(f"'{robot_model}' == '{FULL_KIT}' or '{robot_model}' == '{STARTER_KIT_LEFT}'")
        ),
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

    forward_torque_limit_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_torque_limit_controller', '-c', '/controller_manager'],
    )

    forward_speed_limit_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_speed_limit_controller', '-c', '/controller_manager'],
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
            on_exit=[rviz_spawner],
        ),
    )

    gazebo_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("reachy_gazebo"), '/launch', '/gazebo.launch.py']),
        launch_arguments={'robot_config': f'{robot_model}'}.items()
    )
    # For Gazebo simulation, we should not launch the controller manager (Gazebo does its own stuff)

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                neck_forward_position_controller_spawner,
                r_arm_forward_position_controller_spawner,
                l_arm_forward_position_controller_spawner,
                antenna_forward_position_controller_spawner,
                gripper_forward_position_controller_spawner,
                forward_torque_controller_spawner,
                forward_torque_limit_controller_spawner,
                forward_speed_limit_controller_spawner,
                pid_controller_spawner,
                forward_fan_controller_spawner,
            ],
        ),
    )

    kinematics_spawner = Node(
        package='reachy_kdl_kinematics',
        executable='reachy_kdl_kinematics',
    )

    gripper_safe_controller_spawner = Node(
        package='gripper_safe_controller',
        executable='gripper_safe_controller',
        arguments=['--controllers-file', robot_controllers]
    )

    fake_camera_spawner = Node(
        package='reachy_fake',
        executable='fake_camera',
        condition=IfCondition(fake_arg),
    )

    fake_zoom_node = Node(
        package='reachy_fake',
        executable='fake_zoom',
        condition=IfCondition(
            PythonExpression(f"{fake} or {gazebo}"),
        ),
    )

    return [
        *((control_spawner,) if not gazebo else
          (SetUseSimTime(True),  # does not seem to work...
           gazebo_spawner)),
        fake_camera_spawner,
        fake_zoom_spawner,
        robot_state_publisher_spawner,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        kinematics_spawner,
        gripper_safe_controller_spawner,
        sdk_server_spawner,
        sdk_camera_server_spawner,
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
