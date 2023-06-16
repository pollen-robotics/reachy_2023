# Gazebo config files for Reachy 2023

## Getting started

```bash
ros2 launch reachy_bringup reachy_gz.launch.py
```

Then we can launch ReachySDK with:

```bash
ros2 launch reachy_sdk_server reachy_sdk_server.launch.py
ros2 launch reachy_sdk_server camera_server.launch.py
```

## ReachySDK compatibility

* Access to the joints (only **goal_position** and **present_position**)
* Access to the simulated force sensors (**l_force_gripper** and **r_force_gripper**)
* Access to the cameras (only the images with no zoom control, an extra simulated rgbd camera can be activated in the launch)


## Install

```bash
sudo apt install ros-humble-gazebo-ros2-control ros-humble-compressed-image-transport
```

---
This package is part of the ROS2-based software release of the version 2023 of Reachy.

Visit [pollen-robotics.com](https://pollen-robotics.com) to learn more or join our [Dicord community](https://discord.com/invite/vnYD6GAqJR) if you have any questions or want to share your ideas.
