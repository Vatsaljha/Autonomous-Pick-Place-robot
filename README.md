# Autonomous Pick-and-Place Robot

## Overview

A ROS 2 Humble autonomous pick-and-place robot using TurtleBot3 Waffle Pi, SLAM, Nav2, custom ArUco vision, and OpenMANIPULATOR-X in Gazebo Classic.

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

![Project Architecture](screenshots/architecture.png)

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

## How the Robot Works

### 1. Gazebo Simulation

The complete robot is simulated in Gazebo Classic.

The simulated platform consists of:

- TurtleBot3 Waffle Pi
- Pi Camera
- LiDAR
- OpenMANIPULATOR-X
- Gripper
- Home Service Challenge environment
- ArUco markers
- Pick object

Gazebo provides the robot physics, sensors, environment, and actuator simulation.

---

### 2. SLAM and Localization

SLAM Toolbox processes LiDAR data from:

```text
/scan and creates a map of the environment.

The robot uses the generated map together with odometry and TF to estimate its position.

The resulting map is used by Nav2 for autonomous navigation.

###3. Nav2 Navigation

Nav2 is responsible for long-distance navigation.

The mission manager sends a navigation goal to:

/navigate_to_pose

Nav2 then:

Plans a path
Avoids obstacles
Uses the map for navigation
Controls the mobile base
Brings the robot to the pickup area

Nav2 is used for the large-scale movement rather than manually driving the robot toward the marker.

###4. Custom ArUco Detection

After the robot reaches the pickup area, the camera is used for precise target detection.

The custom detector subscribes to:

/pi_camera/image_raw

Camera calibration is obtained from:

/pi_camera/camera_info

The detector uses OpenCV ArUco detection with:

DICT_5X5_50

The system detects multiple markers in the environment, but the mission manager specifically selects:

Target ID = 7

This prevents other visible markers from becoming the pickup target.

###5. ArUco Pose Estimation

After detecting marker ID 7, the system estimates its position relative to the camera.

The important values are:

X = horizontal position of the marker
Z = forward distance from the camera

For example:

X < 0

means the target is to one side of the camera.

X > 0

means the target is to the opposite side.

The controller uses this information to align the robot with the marker.

###6. Target Centering

The robot does not immediately drive forward after detecting ID 7.

First, it centers the marker in the camera view.

The controller continuously reduces the horizontal error:

X → 0

When the marker is sufficiently close to the image center, the robot switches to forward motion.

This prevents the robot from approaching the marker diagonally.

###7. Straight-Line Final Approach

Once the target is centered, the robot moves forward.

During this stage:

linear velocity > 0
angular velocity = 0

The camera continuously measures the marker distance.

The robot stops when the calibrated target distance reaches approximately:

Z = 0.15 m

This provides a more precise final position than relying only on the Nav2 goal.

###8. LiDAR Safety

The camera provides precise target positioning, while LiDAR provides collision protection.

The local and global costmaps use:

/scan

to detect obstacles.

The final visual controller also checks the front LiDAR region before moving forward.

The robot stops when a persistent unsafe obstacle condition is detected.

This creates two layers of protection:

Camera
  ↓
Precise target positioning

LiDAR
  ↓
Collision protection

###9. Pickup Position

When the robot reaches:

Z ≈ 0.15 m

the mobile base stops.

The target marker is no longer used to control the mobile base during manipulation.

The mission manager then starts the arm and gripper sequence.

###10. Gripper Open

The gripper is controlled through:

/gripper_controller/gripper_cmd

The gripper opens before the arm moves toward the object.

This creates enough space for the object to enter between the fingers.

###11. OpenMANIPULATOR-X Positioning

The OpenMANIPULATOR-X is controlled through:

/arm_controller/follow_joint_trajectory

The calibrated pickup configuration used in the current simulation is:

joint1 = 0.0
joint2 = 0.95
joint3 = -0.95
joint4 = 0.0

The arm moves to this configuration using a joint trajectory.

###12. Gripper Closing

After the arm reaches the pickup pose, the gripper closes around the object.

The sequence is:

Gripper OPEN
      ↓
Arm moves to pickup pose
      ↓
Gripper CLOSE

The arm then moves to a higher configuration to lift the object.

###13. Object Lift

After closing the gripper, the arm moves upward.

This is important because the object must clear the wall and surrounding environment before the mobile robot starts moving again.

The mobile base remains stopped during the manipulation sequence.

##14. Return Navigation

After the object is lifted, the mission manager sends the robot back to the predefined home pose using Nav2.

The robot again uses:

SLAM + Nav2 + LiDAR

for autonomous navigation.

This means the return journey is not manually teleoperated.


Results

The implemented system successfully demonstrates:

- Autonomous navigation to the pickup area
- Detection of the required ArUco marker
- Target centering using camera feedback
- Close-range approach to the marker
- Arm positioning for pickup
- Gripper open/close operation
- Object lifting
- Autonomous return to the home position

Current Limitations

- The implementation is currently validated in Gazebo Classic simulation.
- The mission uses a predefined target marker ID (ID 7).
- Pickup and placement depend on the simulated object's collision/contact behavior.
- The system has been tuned for the current Gazebo challenge environment.
