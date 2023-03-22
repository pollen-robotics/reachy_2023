source /opt/ros/humble/setup.bash
source $HOME/reachy_ws/install/setup.bash
source /usr/share/gazebo/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=~/ros2_install
export LC_NUMERIC="en_US.UTF-8"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source "$HOME/.cargo/env"

ros2 launch reachy_bringup reachy.launch.py start_sdk_server:=true