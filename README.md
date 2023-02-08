# Installation
This guide is meant for an Ubuntu 22.04
## Dependencies
### ROS2 Humble
https://docs.ros.org/en/humble/Installation.html

### Rust
```commandline
curl https://sh.rustup.rs -sSf | sh
```
https://doc.rust-lang.org/cargo/getting-started/installation.html


## Reachy WorkSpace
### Create Reachy_2023 env

```commandline
mkdir ~/reachy_ws && cd ~/reachy_ws
mkdir src && cd src
git clone https://github.com/pollen-robotics/reachy_2023.git
```

## Reachy SDK API
```commandline
mkdir ~/dev && cd ~/dev
git clone https://github.com/pollen-robotics/reachy-sdk-api.git
cd ~/dev/reachy-sdk-api/python
pip3 install -e .
pip3 install scipy
```

## Install controllers
```commandline
cd ~/dev/
git clone https://github.com/pollen-robotics/reachy_2023_controllers_rs.git
cd ~/dev/reachy_2023_controllers_rs
./ros_hwi_install.sh $HOME/reachy_ws/src/reachy_2023
```

### Build Reachy_2023 env

```commandline
cd ~/reachy_ws
sudo ./src/reachy_2023/dependencies.sh
pip install -r ./src/reachy_2023/requirements.txt
colcon build --symlink-install
```

It could be nice to add both file to user .bashrc
```text
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
```

### Gazebo

### Nav