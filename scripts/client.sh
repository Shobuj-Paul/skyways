#!/usr/bin/env bash

source /opt/ros/foxy/setup.bash
source ~/colcon_ws/install/local_setup.bash
ros2 run skyways drone_client $DRONE_ID
