# TurtleBot3 Autonomous Pick-and-Place Robot

## Overview

A custom ROS 2 Humble implementation of an autonomous TurtleBot3 Home Service Challenge in Gazebo Classic.

The project uses a TurtleBot3 Waffle Pi equipped with a Pi Camera and OpenMANIPULATOR-X.

## Key Features

- Autonomous navigation using SLAM and Nav2
- Custom ArUco detection using OpenCV
- Target-specific marker identification (ID 7)
- Camera-based visual target centering
- Straight-line visual final approach
- LiDAR-based collision protection
- OpenMANIPULATOR-X joint trajectory control
- Autonomous gripper open/close control
- Autonomous object pickup and lift
- Nav2-based return-to-home navigation

## Screenshots

### Gazebo Simulation

![Gazebo World](screenshots/gazebo_world.png)

### ArUco Detection

![ArUco Detection](screenshots/aruco_detection.png)

### SLAM and Nav2

![SLAM and Nav2](screenshots/nav2_slam.png)

### Autonomous Pick and Return

![Autonomous Pick and Return](screenshots/autonomous_pick_place.png)

### Project Architecture

![Project Architecture] (screenshots/architecture.png)

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

## Results

The implemented system successfully demonstrates:

- Autonomous navigation to the pickup area
- Detection of the required ArUco marker
- Target centering using camera feedback
- Close-range approach to the marker
- Arm positioning for pickup
- Gripper open/close operation
- Object lifting
- Autonomous return to the home position

## Current Limitations

- The implementation is currently validated in Gazebo Classic simulation.
- The mission uses a predefined target marker ID (ID 7).
- Pickup and placement depend on the simulated object's collision/contact behavior.
- The system has been tuned for the current Gazebo challenge environment.