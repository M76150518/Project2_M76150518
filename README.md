# Project2_M76150518 - Gimbal Lock Demo (ROS 2 Jazzy)

## Overview
This project demonstrates **gimbal lock** by controlling a 2+ joint robot using:
- **Euler angles (roll/pitch/yaw)** via an rqt GUI (shows gimbal lock near pitch ≈ ±90°)
- **Quaternions (x,y,z,w)** via the same rqt GUI (avoids the Euler lock coupling)

## Requirements
- ROS 2 Jazzy
- rviz2
- rqt
- robot_state_publisher

## Build
```bash
cd ~/Project2_M76150518/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

