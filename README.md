# Rviz Plugin

## Abstract
This package is plugin of Rviz2.
support `foxy` of ROS2.

### Install


```
mkdir -p ~/ros2/src
git clone https://github.com/k-nish/rviz_plugins
cd ~/ros2
colcon build --symlink-install
```

### The way of use

```
source /opt/ros/foxy/setup.bash
source ~/ros2/install/setup.bash
ros2 run rviz2 rviz2
```

press `Panel` button on the menu bar and select YesNoPublisher.
