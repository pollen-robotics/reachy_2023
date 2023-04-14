#!/bin/bash
# source ROS2 Foxy setup file.
# shellcheck disable=SC1091


# Env setup
source /home/reachy/reachy_ws/src/reachy_2023/reachy_utils/.reachy.bashrc


zuuu_model=$(reachy-identify-zuuu-model 2>&1)
# Start the ROS2 launch file
if [ $zuuu_model != none ];
then
    echo "Starting mobile base launch file."
    ros2 launch mobile_base_sdk_server run_mobile_base_sdk_server_and_hal.launch.py
else
    echo "No mobile base found in this Reachy model."
fi
