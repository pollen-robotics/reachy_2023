# Reachy Moveit2 configuration package

## System dependencies

sudo apt-get install ros-humble-moveit-msgs
sudo apt remove ros-$ROS_DISTRO-moveit*

Then follow this guide BUT before compiling delete some folders to only have these folders after the 'vcs import < moveit2_tutorials/moveit2_tutorials.repos' step:

launch_param_builder
moveit2
moveit2_tutorials
moveit_resources
moveit_task_constructor
moveit_visual_tools
reachy_2023
rosparam_shortcuts
srdfdom

The guide:
https://moveit.ros.org/install-moveit2/source/


OR THIS SEEMS TO WORK ON SIM SIM'S COMPUTER??
- ros-humble-moveit
- ros-humble-moveit-msgs

## Startup

### In Fake mode

- ros2 launch reachy_moveit_config reachy_moveit_fake.launch.py

### In Gazebo mode

- ros2 launch reachy_moveit_config reachy_moveit_gazebo.launch.py
