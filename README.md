# TurtleBot3 Autonomous Home Service Challenge

## Overview

A custom ROS 2 Humble implementation of an autonomous TurtleBot3 Home Service Challenge in Gazebo Classic.

The project uses a TurtleBot3 Waffle Pi equipped with a Pi Camera and OpenMANIPULATOR-X.

### Main Technologies

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic
- TurtleBot3 Waffle Pi
- OpenMANIPULATOR-X
- SLAM Toolbox
- Nav2
- LiDAR
- OpenCV ArUco
- Python
- ROS 2 Actions

---

## Autonomous Mission

The robot performs the following sequence:

```text
Home
  |
  v
Nav2 navigation
  |
  v
Pickup station
  |
  v
Detect ArUco ID 7
  |
  v
Center target
  |
  v
Straight visual approach
  |
  v
Stop at calibrated distance
  |
  v
Open gripper
  |
  v
Move arm to pick position
  |
  v
Close gripper
  |
  v
Lift object
  |
  v
Nav2 return home
  |
  v
Mission complete