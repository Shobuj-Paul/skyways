# Drone Skyways

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

- Clone ROS 1 repository in `catkin_ws/src` and follow these instructions to build your workspace.
```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws/src
git clone https://github.com/Shobuj-Paul/skyways_ros1.git
cd ..
catkin build
```

- Clone ROS 2 repository in `colcon_ws/src` and follow these instructions to build your workspace.
```bash
source /opt/ros/foxy/setup.bash
cd ~/colcon_ws/src
git clone https://github.com/Shobuj-Paul/skyways_ros2.git
cd ..
colcon build --symlink-install
```

- Next we need to create aliases for easy sourcing of workspaces. Bash scripts for these have been created in the `scripts` folder. Change the workspace names and paths accordingly if you have created them in a different directory.

- Add these aliases to your ~/.bashrc file. With this you will be able to source your workspaces (both overlay and underlay) with commands `noetic` (for ROS 1 and catkin_ws) and `foxy` (for ROS 2 and colcon_ws).
```bash
alias foxy='source ~/colcon_ws/src/skyways_ros2/scripts/foxy.sh'
alias noetic='source ~/catkin_ws/src/skyways_ros1/scripts/noetic.sh'
```

- Set an environment variable in your `~/.bashrc` file to avoid explicity mentioning it everytime you run you client or control codes. 
```bash
export DRONE_ID="<name of your drone>"
```
You can check your variable using this command.
```bash
echo $DRONE_ID
```

## Drone Flight
