from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, TimerAction, \
    OpaqueFunction, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart, OnExecutionComplete
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node, SetUseSimTime, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml
import os

FULL_KIT, STARTER_KIT_RIGHT, STARTER_KIT_LEFT, HEADLESS, MINI = 'full_kit', 'starter_kit_right', 'starter_kit_left', 'headless', 'mini'
STARTER_KIT_RIGHT_NO_HEAD = 'starter_kit_right_no_head'

REACHY_CONFIG_MODEL = "model"
REACHY_CONFIG_NECK_ORBITA_ZERO = "neck_orbita_zero"
REACHY_CONFIG_TOP = "top"
REACHY_CONFIG_BOTTOM = "bottom"
REACHY_CONFIG_MIDDLE = "middle"


# Before launching each node, scan the usb2ax to check if there are any missing motors
from reachy_utils.discovery import get_missing_motors_reachy
get_missing_motors_reachy(check_service=False)


class ReachyConfig:
    def __init__(self, config_file_path='~/.reachy.yaml'):

        config_file = os.path.expanduser(config_file_path)
        with open(config_file) as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

            # Robot model
            if config[REACHY_CONFIG_MODEL] in [FULL_KIT, STARTER_KIT_RIGHT, STARTER_KIT_LEFT, HEADLESS, MINI, STARTER_KIT_RIGHT_NO_HEAD]:
                self.model = config[REACHY_CONFIG_MODEL]
            else:
                raise ValueError('Bad robot model "{}". Expected values are {}'.format(
                    config[REACHY_CONFIG_MODEL],[FULL_KIT, STARTER_KIT_RIGHT, STARTER_KIT_LEFT, HEADLESS, MINI, STARTER_KIT_RIGHT_NO_HEAD]))

            # orbita zero
            try:
                self.neck_orbita_zero_top = config[REACHY_CONFIG_NECK_ORBITA_ZERO][REACHY_CONFIG_TOP]
                self.neck_orbita_zero_middle = config[REACHY_CONFIG_NECK_ORBITA_ZERO][REACHY_CONFIG_MIDDLE]
                self.neck_orbita_zero_bottom = config[REACHY_CONFIG_NECK_ORBITA_ZERO][REACHY_CONFIG_BOTTOM]
            except KeyError as e:
                raise KeyError("neck_orbita_zero key not found :: {}".format(e))

    def __str__(self):
        return "robot_model".ljust(25, ' ') + "{}\n".format(self.model) + \
            "neck_orbita_zero_top".ljust(25, ' ') + "{}\n".format(self.neck_orbita_zero_top) + \
            "neck_orbita_zero_middle".ljust(25, ' ') + "{}\n".format(self.neck_orbita_zero_middle) + \
            "neck_orbita_zero_bottom".ljust(25, ' ') + "{}\n".format(self.neck_orbita_zero_bottom)


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

    # Robot config
    reachy_config = ReachyConfig()
    LogInfo(msg="Reachy config : \n{}".format(reachy_config)).execute(context=context)

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
            f'robot_config:={reachy_config.model}',
            ' ',
            'orbita_hardware_zero:="{}, {}, {}"'.format(
                reachy_config.neck_orbita_zero_top,
                reachy_config.neck_orbita_zero_middle,
                reachy_config.neck_orbita_zero_bottom),
            ' ',
        ]
    )  # To be cleaned on issue #92
    # print(robot_description_content.perform(context=context))

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
    }

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('reachy_bringup'),
            'config',
            f'reachy_{reachy_config.model}_controllers.yaml',
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

    sdk_server_node = Node(
        package='reachy_sdk_server',
        executable='reachy_sdk_server',
        output='both',
        arguments=[reachy_config.model],
        condition=IfCondition(start_sdk_server_rl),
    )

    camera_publisher_node = Node(
        package='camera_controllers',
        executable='camera_publisher',
        output='both',
        condition=IfCondition(
            PythonExpression(
                f"not {fake_py} and not {gazebo_py} and '{reachy_config.model}' not in ['{HEADLESS}', '{STARTER_KIT_RIGHT_NO_HEAD}']"
            )),
    )

    camera_focus_node = Node(
        package='camera_controllers',
        executable='camera_focus',
        output='both',
        condition=IfCondition(
            PythonExpression(
                f"not {fake_py} and not {gazebo_py} and '{reachy_config.model}' not in ['{HEADLESS}', '{STARTER_KIT_RIGHT_NO_HEAD}']"
            )),
    )

    camera_zoom_node = Node(
        package='camera_controllers',
        executable='camera_zoom_service',
        output='both',
        condition=IfCondition(
            PythonExpression(
                f"not {fake_py} and not {gazebo_py} and '{reachy_config.model}' not in ['{HEADLESS}', '{STARTER_KIT_RIGHT_NO_HEAD}']"
            )),
    )

    sdk_camera_server_node = Node(
        package='reachy_sdk_server',
        executable='camera_server',
        output='both',
        condition=IfCondition(PythonExpression(
                f"{start_sdk_server_py} and '{reachy_config.model}' not in ['{HEADLESS}', '{STARTER_KIT_RIGHT_NO_HEAD}']"
            )),
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
        condition=IfCondition(start_rviz_rl),
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

    neck_forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['neck_forward_position_controller', '-c', '/controller_manager'],
        condition=IfCondition(
            PythonExpression(
                f"'{reachy_config.model}' != '{HEADLESS}'")
        )
    )

    r_arm_forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['r_arm_forward_position_controller', '-c', '/controller_manager'],
        condition=IfCondition(
            PythonExpression(
                f"'{reachy_config.model}' in ['{STARTER_KIT_RIGHT}', '{FULL_KIT}', '{HEADLESS}']")
        )
    )

    l_arm_forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['l_arm_forward_position_controller', '-c', '/controller_manager'],
        condition=IfCondition(
            PythonExpression(
                f"'{reachy_config.model}' in ['{STARTER_KIT_LEFT}', '{FULL_KIT}', '{HEADLESS}']")
        ),
    )

    antenna_forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['antenna_forward_position_controller', '-c', '/controller_manager'],
        condition=IfCondition(
            PythonExpression(
                f"'{reachy_config.model}' not in ['{HEADLESS}', '{STARTER_KIT_RIGHT_NO_HEAD}']")
        )
    )

    gripper_forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_forward_position_controller', '-c', '/controller_manager'],
        condition=IfCondition(
            PythonExpression(
                f"'{reachy_config.model}' != '{MINI}'")
        ),
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

    forward_pid_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_pid_controller', '-c', '/controller_manager'],
    )

    forward_fan_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_fan_controller', '-c', '/controller_manager'],
    )

    fan_controller_spawner = Node(
        package='fans_controller',
        executable='fans_controller',
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
        launch_arguments={'robot_config': f'{reachy_config.model}'}.items()
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
                forward_pid_controller_spawner,
                forward_fan_controller_spawner,
                fan_controller_spawner,
                kinematics_node
            ],
        ),
    )

    delay_sdk_server_after_kinematics = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=kinematics_node, goal_state='inactive',
            entities=[sdk_server_node],
        )
    )

    gripper_safe_controller_node = Node(
        package='gripper_safe_controller',
        executable='gripper_safe_controller',
        arguments=['--controllers-file', robot_controllers],
        condition=IfCondition(
                    PythonExpression(
                        f"'{reachy_config.model}' != '{MINI}'")
        ),
    )

    fake_camera_node = Node(
        package='reachy_fake',
        executable='fake_camera',
        condition=IfCondition(fake_rl),
    )

    fake_zoom_node = Node(
        package='reachy_fake',
        executable='fake_zoom',
        condition=IfCondition(
            PythonExpression(f"{fake_py} or {gazebo_py}"),
        ),
    )

    return [
        *((control_node,) if not gazebo_py else
          (SetUseSimTime(True),  # does not seem to work...
           gazebo_node)),
        fake_camera_node,
        fake_zoom_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        gripper_safe_controller_node,
        delay_sdk_server_after_kinematics,
        camera_publisher_node,
        camera_focus_node,
        camera_zoom_node,
        sdk_camera_server_node,
        dynamic_state_router_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        # Needed by camera publisher - See: https://github.com/ros2/rosidl_python/issues/79
        SetEnvironmentVariable('PYTHONOPTIMIZE', '1'),

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
        OpaqueFunction(function=launch_setup)
    ])

# TODO use a OnProcessIO to check whether every node has sent its 'OK' message and log accordingly ?
