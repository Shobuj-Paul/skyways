#!/usr/bin/env bash

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun skyways geofence_rectangle $DRONE_ID
