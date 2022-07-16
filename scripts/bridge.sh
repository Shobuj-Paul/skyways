#!/usr/bin/env bash

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source /opt/ros/foxy/setup.bash
source ~/colcon_ws/install/local_setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
