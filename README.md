# Installation
This guide is meant for an Ubuntu 22.04
## Environment
### ROS2 Humble
https://docs.ros.org/en/humble/Installation.html

We use Cyclone DDS, here is a [link](https://docs.ros.org/en/humble/Installation/DDS-Implementations/Working-with-Eclipse-CycloneDDS.html) 
for reference, but everything to install it is already included in current guide.

```commandline
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Rust
```commandline
curl https://sh.rustup.rs -sSf | sh
```

You may add to your bashrc, or source it anytime you want to build `reachy_ws`
```commandline
source "$HOME/.cargo/env"
```

https://doc.rust-lang.org/cargo/getting-started/installation.html


## Reachy WorkSpace
### Create Reachy_2023 env

```commandline
mkdir ~/reachy_ws && cd ~/reachy_ws
mkdir src && cd src
git clone https://github.com/pollen-robotics/reachy_2023.git
```



### Build Reachy_2023 env

```commandline
cd ~/reachy_ws
./src/reachy_2023/dependencies.sh
pip install -r ./src/reachy_2023/requirements.txt
colcon build --symlink-install
```

Now you can add the following to your bashrc
```commandline
source ~/ros_ws/install/setup.bash
```

### Gazebo

### Nav


## Reachy SDK API

```commandline
mkdir ~/dev && cd ~/dev
git clone https://github.com/pollen-robotics/reachy-sdk-api.git
cd ~/dev/reachy-sdk-api/python
pip3 install -e .
pip3 install scipy
```

## Examples

### Fake

```commandline
ros2 launch reachy_bringup reachy.launch.py  fake:=true start_rviz:=true
```

If you see a full_kit robot (torso, head and arms) inside rviz, then it should mean that most stuff
went right, or at least that not everything went wrong.
