# robotics-project
Simulation of a robotic arm on a mobile base using ros2 humble and gazebo 11.

![screenshot](https://github.com/ronejfourn/robotics-project/blob/main/screenshot.png)

## Installation
First, follow the instructions provided [here](https://github.com/IFRA-Cranfield/ros2_RobotSimulation/tree/humble?tab=readme-ov-file#ros20-humble-environment-set-up) upto step 6 and skip step 5.
### Install necessary packages
```sh
sudo apt install python3-colcon-common-extensions \
                 ros-humble-slam-toolbox ros-humble-twist-mux \
                 ros-humble-navigation2 ros-humble-nav2-bringup
```

### Create and build the workspace
```sh
mkdir -p ~/proj_ws/src
cd ~/proj_ws/src
git clone https://github.com/ronejfourn/robotics-project
git clone -b humble https://github.com/Box-Robotics/ros2_numpy
cd ..
colcon build --symlink-install
```

## Usage
```sh
source ~/proj_ws/install/setup.sh
cd ~/proj_ws/src/robotics-project
ros2 launch robotics-project launch.py
```

## Issues
The `navigation_launch.py` and `localization_launch.py` scripts may fail to launch when using `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. To fix this, remove the `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` line for your `.bashrc` and rebuild the workspace.
