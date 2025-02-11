```sh
mkdir -p prj_ws/src
cd prj_ws/src
git clone https://github.com/ronejfourn/robotics-project the_arm
cd ..
colcon build --symlink-install
source ./install/setup.sh
ros2 launch the_arm launch.py
```
