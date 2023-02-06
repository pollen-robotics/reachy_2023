## Context
Temporary repository to build the mechanical description of the zuu mobile base.
Eventually, will be merged into :
https://github.com/pollen-robotics/reachy2021_ros2_control/tree/main/reachy_description

## Setup
Add to your bashrc:
```
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
export ZUUU_MAP_NAME=hospital
```

Create a /meshes folder:
```
mkdir -p zuuu_description/meshes
cd zuuu_description/meshes
```

Download the mesh used: [link](https://drive.google.com/file/d/1y3KqgaIK0916n6ELnhmQw_U-jSFYrvL3/view?usp=sharing%29).

## Usage
For a simulated environment:
```
ros2 launch zuuu_description full_simulation_navigation.launch.py
```

To create a map on the physical robot:
```
ros2 launch zuuu_description zuuu_full_navigation.launch.py
```

To start the navigation on the physical robot, if your map name is jean_jaures_haut.yaml, add this to your bashrc:
```
export ZUUU_MAP_NAME=jean_jaures_haut
```
Then:
```
ros2 launch zuuu_description zuuu_full_navigation.launch.py
```

## List of launch files and uses
Launch files starting with "zuuu" are intended to be used on the physical robot.
There are a lot of them for convenience but their content is pretty straightforward.

### description_bringup
Nodes: 
joint_state_broadcaster_spawner
robot_state_publisher_node
controller_manager_node

Default parameters for robot_description_content :
'use_gazebo:=false',
'use_fake_components:=false',
'use_fixed_wheels:=true',
'use_ros_control:=false',

### gazebo_simulation
Nodes: 
joint_state_broadcaster_spawner,
robot_state_publisher_node,
spawn_entity,
controller_manager_node,

Default parameters for robot_description_content :
'use_gazebo:=true',
'use_fake_components:=false',
'use_fixed_wheels:=true',
'use_ros_control:=true',


### full_simulation_navigation
Launches: 
gazebo_simulation
navigation
rviz_navigation

### mapping
Nodes:
slam_toolbox

### navigation
Launches:
Nav2's bringup_launch

### rviz_bringup
Nodes:
rviz2

### zuuu_bringup
Nodes:
Rviz

Launches:
zuuu_rplidar_s2_launch
description_bringup
hal_launch

### zuuu_bringup_low_level_only
Launches:
zuuu_rplidar_s2_launch
hal_launch

### zuuu_bringup_no_rviz
Launches:
zuuu_rplidar_s2_launch
description_bringup
hal_launch


### zuuu_full_mapping
Launches:
zuuu_bringup
mapping

### zuuu_full_navigation
Launches:
zuuu_bringup
navigation