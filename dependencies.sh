#!/bin/bash

sudo apt-get install -y python3-pip
sudo apt-get install -y ament-cmake
sudo apt-get install -y python3-pykdl
sudo apt-get install -y ros-humble-ros2-control ros-humble-joint-state-publisher-gui ros-humble-ros2-controllers ros-humble-xacro ros-humble-rmw-cyclonedds-cpp ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control  ros-humble-tf-transformations
sudo apt-get install -y libclang-dev
cargo install cargo-ament-build