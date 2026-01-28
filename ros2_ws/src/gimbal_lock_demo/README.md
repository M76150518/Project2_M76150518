# Project2_M76150518 — Gimbal Lock Demo (ROS 2 Jazzy)

## Overview
This project demonstrates **gimbal lock** using a custom robot model (URDF with at least two joints).
You can control the robot using:
- **Euler angles (roll/pitch/yaw)** via an rqt GUI
- **Quaternions (x,y,z,w)** via the same GUI

Near **pitch ≈ 90°**, Euler angles exhibit **gimbal lock**, where yaw and roll become coupled.
Quaternions avoid this singularity.

---

## Requirements
- ROS 2 Jazzy
- rviz2
- rqt
- rqt_gui / rqt_gui_py

---

## Build
```bash
cd ~/Project2_M76150518/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
