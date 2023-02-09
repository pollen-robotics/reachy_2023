#!/bin/bash

apt-get install -y ament-cmake
apt-get install -y python3-pykdl
apt-get install -y ros-humble-ros2-control ros-humble-joint-state-publisher-gui ros-humble-ros2-controllers ros-humble-xacro
apt-get install -y libclang-dev
cargo install cargo-ament-build