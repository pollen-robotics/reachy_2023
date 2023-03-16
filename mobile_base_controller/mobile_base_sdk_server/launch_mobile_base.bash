#!/bin/bash
# source ROS2 Foxy setup file.
# shellcheck disable=SC1091
source /home/reachy/reachy_tips/config/reachy_ros_config

IDENTIFY_CMD=$HOME/.local/bin/reachy-identify-zuuu-model
zuuu_model=`$IDENTIFY_CMD`

# Start the ROS2 launch file
zuuu_model=$(reachy-identify-zuuu-model)

if [ $zuuu_model != none ];
then
    echo "Starting mobile base launch file."
    ros2 launch mobile_base_sdk_server run_mobile_base_sdk_server_and_hal.launch.py
else
    echo "No mobile base found in this Reachy model."
fi
