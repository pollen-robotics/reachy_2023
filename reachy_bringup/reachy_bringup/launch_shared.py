from launch.actions import DeclareLaunchArgument

FULL_KIT, STARTER_KIT_RIGHT, STARTER_KIT_LEFT = 'full_kit', 'starter_kit_right', 'starter_kit_left'


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