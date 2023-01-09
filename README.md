# Drone Skyways

[![GitHub issues](https://img.shields.io/github/issues/Shobuj-Paul/skyways?color=red&label=Issues&style=flat)](https://github.com/Shobuj-Paul/skyways/issues)
[![GitHub license](https://img.shields.io/github/license/Shobuj-Paul/skyways?color=green&label=License&style=flat)](https://github.com/Shobuj-Paul/skyways/blob/main/LICENSE)
![Ubuntu](https://img.shields.io/badge/Ubuntu%2020.04-%E2%9C%94-blue)
![ROS Foxy](https://img.shields.io/badge/ROS%20Foxy-%E2%9C%94-blue)
![Build Status](https://github.com/Shobuj-Paul/skyways/actions/workflows/ros2-ci.yml/badge.svg)

## ROS Installation
- Ensure that your onboard computer supports 64-bit Ubuntu 20.04, either server or desktop.
- Download ROS 1 (Noetic) and ROS 2 (Foxy) from the links given below.

    ROS 1 (Noetic): [http://wiki.ros.org/noetic/Installation/Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu)

    ROS 2 (Foxy): [https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

> Note: ROS 1 installation is required only on onboard computer (or for development). For running a Ground Station, only ROS 2 is required.

- Install catkin and colcon build systems.
```bash
sudo apt install python3-catkin-tools -y
sudo apt install python3-colcon-common-extensions -y
```

- Install `ros1_bridge` for ROS 2 - ROS 1 message conversions
```bash
sudo apt install ros-foxy-ros1-bridge
```

- Install MAVROS for ROS 1 communication with PX4 Autopilot.
```bash
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
```
- After this install GeographicLib Datasets.
```bash
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh
```
> NOTE: In case you are unable to install GeographicLib Datasets, check you clock. Time on the computer should be proper. You can set time using this command.
```bash
sudo date -s "$(wget -qSO- --max-redirect=0 google.com 2>&1 | grep Date: | cut -d' ' -f5-8)Z"
```

## Environment Setup
- Create two workspaces, one for ROS 1 and another for ROS 2.
```bash
mkdir -p catkin_ws/src
mkdir -p colcon_ws/src
```

- Clone the repository and checkout the `ros-1` branch in `catkin_ws/src` and follow these instructions to build your workspace.
```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws/src
git clone https://github.com/Shobuj-Paul/skyways.git
cd skyways
git checkout ros-1
cd ../..
catkin build
```

- Clone the repository and checkout `ros-2` branch (default branch so you can leave as it is) in `colcon_ws/src` and follow these instructions to build your workspace.
```bash
source /opt/ros/foxy/setup.bash
cd ~/colcon_ws/src
git clone https://github.com/Shobuj-Paul/skyways.git
cd skyways
git checkout ros-2
cd ../..
colcon build --symlink-install
```

- Next we need to create aliases for easy sourcing of workspaces. Bash scripts for these have been created in the `scripts` folder. Change the workspace names and paths accordingly if you have created them in a different directory.

- Add these aliases to your ~/.bashrc file. With this you will be able to source your workspaces (both overlay and underlay) with commands `noetic` (for ROS 1 and catkin_ws) and `foxy` (for ROS 2 and colcon_ws).
```bash
alias foxy='source ~/colcon_ws/src/skyways/scripts/foxy.sh'
alias noetic='source ~/catkin_ws/src/skyways/scripts/noetic.sh'
```

- Set an environment variable in your `~/.bashrc` file to avoid explicity mentioning it everytime you run you client or control codes. 
```bash
export DRONE_ID="<name of your drone>"
```
You can check your variable using this command.
```bash
echo $DRONE_ID
```

## Drone Flight Setup
> This manual setup is no longer necessary, you can follow the instructions given in the next part to automate this process.

- Make sure that the onboard computer has permission to read and write to /dev/ttyACM0 port. If that is not the case, you can set permission using the following command.
```bash
sudo chmod 777 /dev/ttyACM0
```
This is done manually and is not permanent, each login requires you to do this manually. It is better to add yourself to dialout group.
```bash
usermod -a -G dialout $USER
sudo adduser $USER
```
For Nvidia development boards this alone does not work. You need to set `udev` rules.
```bash
sudo echo 'ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="09cb", OWNER=$USER, MODE="0777", GROUP="nvidia"' >> /etc/udev/rules.d/50-usb.rules
```
Logout and login again for changes to take effect.
- Open up 4 terminal windows.
- Source ROS 1. Start MAVROS connection to PX4. Make sure the PX4 controller (Pixhawk or Cube) is connected via serial to USB port on the onboard computer.
```bash
noetic
roslaunch skyways px4.launch ID:=$DRONE_ID fcu_url:="/dev/ttyACM0"
```
- Source ROS 1. Run your control code.
```bash
noetic
rosrun skyways setpoint_mission $DRONE_ID
```
- Source ROS 1. Source ROS 2. Run `ros1_bridge`.
```bash
noetic
foxy
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```
- Source ROS 2. Run ROS 2 client for sending flight request.
```bash
foxy
ros2 run skyways drone_client $DRONE_ID
```

## Drone Flight Setup on Startup
- Enable automatic login on Ubuntu from settings.
- Add terminal as a start up process in `Startup Applications` with the following command.
```bash
gnome-terminal
```
- Add the following line to the end of your ~/.bashrc file.
```bash
source ~/colcon_ws/src/skyways/scripts.startup.sh
```
- Reboot. You can check on your Ground Station PC whether the topics are broadcasted via this command.
```bash
foxy
ros2 topic list
```

## Ground Station Command
- Clone the repository in a ROS 2 workspace on your ground station and build.
```bash
source /opt/ros/foxy/setup.bash
mkdir -p colcon_ws/src
cd colcon_ws/src
git clone https://github.com/Shobuj-Paul/skyways.git
cd skyways
git checkout ros-2
cd ../..
colcon build --symlink-install
```
- You can log waypoints from the drone using the following command
```bash
ros2 run skyways waypoint_logger <Drone ID>
```

- Start the ROS 2 server on Ground Station.
```bash
foxy
ros2 run skyways drone_server
```

## Development
- All online flight behaviors need to be programmed in ROS 1 package.
- All offline flight planning needs to programmed in ROS 2 package, within the drone_server.cpp file, `void data(const std::shared_ptr<skyways::srv::DataPacket::Request> request, std::shared_ptr<skyways::srv::DataPacket::Response> response)` function.

## Network
- To set up Wifi hotspot with Alfa Wifi router, connect the router and run the following command.
```bash
nmcli dev wifi hotspot ifname wlx00c0ca980d57 ssid alfa password "abcd1234" 
```