# Gazebo config files for Reachy 2023

## Getting started

```bash
ros2 launch reachy_bringup reachy_gz_launch.py
```

Then we can launch ReachySDK with:

```bash
ros2 launch reachy_sdk_server reachy_sdk_server.launch.py
```

## ReachySDK compatibility

* Access to the joints (only **goal_position** and **present_position**)
* Access to the simulated force sensors (**l_force_gripper** and **r_force_gripper**)
* Access to the cameras (an extra simulated rgbd camera can be activated in the launch)


---
This package is part of the ROS2-based software release of the version 2023 of Reachy.

Visit [pollen-robotics.com](https://pollen-robotics.com) to learn more or join our [Dicord community](https://discord.com/invite/Kg3mZHTKgs) if you have any questions or want to share your ideas.
