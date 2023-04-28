from launch.launch_description import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):
    robot_config = LaunchConfiguration('robot_config', default='full_kit')

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            ])
        ),
        launch_arguments={'verbose': 'true'}.items(),

    )

    # Note: Environment variable GAZEBO_MODEL_PATH is extended as in
    # ROS2 control demos via environment hook https://github.com/ros-controls/ros2_control_demos/tree/master/ros2_control_demo_description/rrbot_description
    # Also see https://colcon.readthedocs.io/en/released/developer/environment.html#dsv-files
    # Gazebo launch scripts append GAZEBO_MODEL_PATH with known paths, see https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ab1ae5c05eda62674b36df74eb3be8c93cdc8761/gazebo_ros/launch/gzclient.launch.py#L26
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", LaunchConfiguration("robot_name"),
        ],
        output="screen"
    )

    fake_gz_interface = Node(
        package="reachy_gazebo",
        executable="fake_gz_interface",
        output="screen",
        parameters=[{'robot_config': f'{robot_config.perform(context)}'}]
    )

    return [
        gazebo,
        fake_gz_interface,
        spawn_entity,
    ]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_name",
                default_value="reachy",
                description="Set robot name."),
            DeclareLaunchArgument(
                "robot_config",
                default_value="full_kit",
                description="Robot configuration."),
            OpaqueFunction(function=launch_setup),
        ]
    )
