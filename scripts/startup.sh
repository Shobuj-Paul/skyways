#!/usr/bin/env bash

# Start MAVROS Connection
gnome-terminal --tab -- ~/colcon_ws/src/skyways/scripts/mavros.sh

# Start ROS-1 Node
gnome-terminal --tab -- ~/colcon_ws/src/skyways/scripts/drone.sh

# Start ROS-2 Client
gnome-terminal --tab -- ~/colcon_ws/src/skyways/scripts/client.sh

# Start ROS-1 Bridge
gnome-terminal --tab -- ~/colcon_ws/src/skyways/scripts/bridge.sh
