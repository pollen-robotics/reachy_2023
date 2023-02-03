# Zuuu ROS2 services

Custom ROS2 services used by Reachy's mobile base ROS2 [HAL](https://github.com/pollen-robotics/zuuu_hal/) and [SDK Server](https://github.com/pollen-robotics/mobile_base_sdk_server).

**ROS2 Version: Foxy**

How to install:

```bash
cd ~/reachy_ws/src
git clone https://github.com/pollen-robotics/zuuu_interfaces.git
cd ~/reachy_ws/
colcon build --packages-select zuuu_interfaces
```

## Services
* **DistanceToGoal.srv** - Return delta x, delta y, delta theta and distance from the last goal position sent using GoToXYTheta. 
* **GetBatteryVoltage.srv** - Get the mobile base's battery voltage.
* **GetOdometry.srv** - Get the mobile base's odometry.
* **GetZuuuMode.srv** - Get the mobile base's drive mode.
* **GoToXYTheta.srv** - Send GoTo instruction to the mobile base.
* **IsGoToFinished.srv** - Return if the mobile base has reached the goal of its last GoTo sent, modulo tolerances along x, y and theta.
* **ResetOdometry.srv** - Reset the mobile base's odometry.
* **SetSpeed.srv** - Send velocities commands to the mobile base.
* **SetZuuuMode.srv** - Set the mobile base's drive mode.
* **SetZuuuSafety.srv** - Disable / enable the mobile base's anti-collision safety provided by the Lidar.

---
This package is part of the July 2022's ROS2-based software release of the mobile base working with Reachy 2021.

Visit [pollen-robotics.com](https://pollen-robotics.com) to learn more or join our [Dicord community](https://discord.com/invite/Kg3mZHTKgs) if you have any questions or want to share your ideas.
