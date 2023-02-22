# Reachy Moveit2 configuration package
## The locales bug
Make sure to add this t you .bashrc:
```
export LC_NUMERIC="en_US.UTF-8"
```
## System dependencies
2 options:
### Installing from repos (easy, short)
```sudo apt-get install ros-humble-moveit ros-humble-moveit-msgs```

### Installing from source (long)
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
https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html

Otherwise, this could help:
https://moveit.ros.org/install-moveit2/source/


## Startup

### In Fake mode

- ros2 launch reachy_moveit_config reachy_moveit_fake.launch.py

### In Gazebo mode

- ros2 launch reachy_moveit_config reachy_moveit_gazebo.launch.py
