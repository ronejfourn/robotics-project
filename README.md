# Installation
```sh
mkdir -p ~/proj_ws/src
cd ~/proj_ws/src
git clone https://github.com/ronejfourn/robotics-project
git clone https://github.com/Box-Robotics/ros2_numpy
cd ..
colcon build --symlink-install
```
# Usage
```sh
source ~/proj_ws/install/setup.sh
cd ~/proj_ws/src/robotics-project
ros2 launch robotics-project launch.py
```
