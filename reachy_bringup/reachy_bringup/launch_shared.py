from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.descriptions import ParameterValue

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


def get_robot_controllers(robot_model):
    return PathJoinSubstitution(
        [
            FindPackageShare('reachy_bringup'),
            'config',
            f'reachy_{robot_model}_controllers.yaml',
        ]
    )


def get_robot_description(robot_model, fake, gazebo):
    return {
        'robot_description': ParameterValue(Command(
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
        ), value_type=str),
    }


fake_launch_arg = DeclareLaunchArgument(
    name='fake',
    default_value='false',
    description='Start on fake_reachy mode with this launch file.',
    choices=['true', 'false']
)

gazebo_launch_arg = DeclareLaunchArgument(
    name='gazebo',
    default_value='false',
    description='Start a fake_hardware with gazebo as simulation tool.',
    choices=['true', 'false']
)

robot_model_launch_arg = DeclareLaunchArgument(
    'robot_model',
    default_value=FULL_KIT,
    description='Choose robot_model configuration. '
                'If a robot_configuration is defined in ~/.reachy.yaml : it WILL BE CHOSEN over any given arg',
    choices=[FULL_KIT, STARTER_KIT_RIGHT, STARTER_KIT_LEFT]
)
