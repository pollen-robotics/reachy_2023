source /opt/ros/humble/setup.bash
source /home/reachy/reachy_ws/install/setup.bash
source /usr/share/gazebo/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=~/ros2_install
export LC_NUMERIC="en_US.UTF-8"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source "$HOME/.cargo/env"
export PATH=$PATH:$HOME/.local/bin
export ROS_DOMAIN_ID=$(grep "export ROS_DOMAIN_ID=" ~/.bashrc | cut -d "=" -f2)
