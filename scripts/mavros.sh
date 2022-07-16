#!/usr/bin/env bash

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch skyways px4.launch ID:=$DRONE_ID fcu_url:=/dev/ttyACM0
