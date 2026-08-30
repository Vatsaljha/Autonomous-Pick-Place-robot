#!/usr/bin/env python3

import math
import os

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus

from cv_bridge import CvBridge

from geometry_msgs.msg import PoseStamped, Twist

from nav2_msgs.action import NavigateToPose

from sensor_msgs.msg import CameraInfo, Image, LaserScan

from std_msgs.msg import Int32MultiArray

from trajectory_msgs.msg import JointTrajectoryPoint

from control_msgs.action import (
    FollowJointTrajectory,
    GripperCommand,
)


class PickMission(Node):

    def __init__(self):
        super().__init__('pick_mission')

        # =========================================================
        # CONFIG FILE
        # =========================================================

        self.config_file = os.path.expanduser(
            '~/turtlebot3_ws/config/pick_config.yaml'
        )

        try:
            with open(self.config_file, 'r') as file:
                config = yaml.safe_load(file)
        except Exception as error:
            self.get_logger().error(
                f'Failed to load config: {error}'
            )
            raise

        # =========================================================
        # TARGET MARKER
        # =========================================================

        self.target_id = int(
            config.get('marker_id', 7)
        )

        # =========================================================
        # PICK STAGING POSE
        # =========================================================

        pick_pose = config['pick_pose']

        self.pick_x = float(
            pick_pose['x']
        )

        self.pick_y = float(
            pick_pose['y']
        )

        self.pick_yaw = float(
            pick_pose['yaw']
        )

        # =========================================================
        # HOME POSE
        # =========================================================

        home_pose = config['home_pose']

        self.home_x = float(
            home_pose['x']
        )

        self.home_y = float(
            home_pose['y']
        )

        self.home_yaw = float(
            home_pose['yaw']
        )

        # =========================================================
        # ARM PICK POSE
        # =========================================================

        arm_pose = config['arm_pose']

        self.arm_pick_positions = [
            float(arm_pose['joint1']),
            float(arm_pose['joint2']),
            float(arm_pose['joint3']),
            float(arm_pose['joint4']),
        ]

        # =========================================================
        # FINAL ARUCO APPROACH
        # =========================================================

        # USER-TESTED VALUE
        #
        # Robot approaches until:
        #
        # camera Z <= 0.15 m
        #
        self.final_target_z = 0.15

        # Target centered when:
        #
        # -0.025 <= X <= +0.025
        #
        self.x_tolerance = 0.025

        # Slow straight approach speed.
        self.approach_speed = 0.04

        # Maximum rotation speed.
        self.max_angular_speed = 0.30

        # =========================================================
        # LIDAR SAFETY
        # =========================================================

        # Narrow front cone because once ID 7 is centered,
        # the robot only moves straight forward.
        self.lidar_half_angle_deg = 15.0

        # Emergency distance.
        #
        # This is deliberately below the previous 0.24 m value
        # because the previous threshold sometimes stopped the
        # robot before the camera reached 0.15 m.
        self.min_lidar_distance = 0.18

        # Require several consecutive unsafe measurements.
        self.safety_stop_count = 0
        self.safety_required_count = 5

        # =========================================================
        # ARUCO
        # =========================================================

        # Marker size used by the Gazebo world.
        self.marker_size = 0.088

        # OpenCV 4.5.x compatible API.
        self.dictionary = cv2.aruco.Dictionary_get(
            cv2.aruco.DICT_5X5_50
        )

        self.parameters = (
            cv2.aruco.DetectorParameters_create()
        )

        # =========================================================
        # STATE
        # =========================================================

        self.state = 'WAITING'

        # Possible states:
        #
        # WAITING
        # NAVIGATING
        # VISUAL_APPROACH
        # ALIGNING
        # APPROACHING
        # OPENING_GRIPPER
        # MOVING_ARM
        # CLOSING_GRIPPER
        # LIFTING
        # RETURNING_HOME
        # MISSION_COMPLETE
        # SAFETY_STOP
        # ERROR

        # =========================================================
        # ARUCO DATA
        # =========================================================

        self.target_seen = False

        # Camera X:
        #   negative = left
        #   positive = right
        self.target_x = None

        # Camera Z:
        #   forward distance
        self.target_z = None

        # =========================================================
        # CAMERA
        # =========================================================

        self.bridge = CvBridge()

        self.camera_matrix = None
        self.dist_coeffs = None

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/pi_camera/camera_info',
            self.camera_info_callback,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            '/pi_camera/image_raw',
            self.image_callback,
            10
        )

        # Existing detector's marker-ID topic.
        self.marker_id_sub = self.create_subscription(
            Int32MultiArray,
            '/aruco/marker_ids',
            self.marker_id_callback,
            10
        )

        # =========================================================
        # LIDAR
        # =========================================================

        self.latest_scan = None

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # =========================================================
        # ROBOT VELOCITY
        # =========================================================

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # =========================================================
        # DEBUG IMAGE
        # =========================================================

        self.debug_pub = self.create_publisher(
            Image,
            '/aruco/mission_debug',
            10
        )

        # =========================================================
        # NAV2
        # =========================================================

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.nav_goal_handle = None

        # =========================================================
        # ARM
        # =========================================================

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        # =========================================================
        # GRIPPER
        # =========================================================

        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd'
        )

        # =========================================================
        # TIMERS
        # =========================================================

        self.start_timer = self.create_timer(
            2.0,
            self.start_mission
        )

        self.control_timer = self.create_timer(
            0.05,
            self.control_loop
        )

        # =========================================================
        # STARTUP LOG
        # =========================================================

        self.get_logger().info(
            '=============================================='
        )

        self.get_logger().info(
            '       AUTONOMOUS PICK AND RETURN'
        )

        self.get_logger().info(
            '=============================================='
        )

        self.get_logger().info(
            f'Target marker ID: {self.target_id}'
        )

        self.get_logger().info(
            f'Pick staging pose: '
            f'x={self.pick_x:.3f}, '
            f'y={self.pick_y:.3f}, '
            f'yaw={self.pick_yaw:.3f}'
        )

        self.get_logger().info(
            f'Home pose: '
            f'x={self.home_x:.3f}, '
            f'y={self.home_y:.3f}, '
            f'yaw={self.home_yaw:.3f}'
        )

        self.get_logger().info(
            f'Final ArUco distance: '
            f'{self.final_target_z:.2f} m'
        )

        self.get_logger().info(
            f'LiDAR safety distance: '
            f'{self.min_lidar_distance:.2f} m'
        )

        self.get_logger().info(
            f'LiDAR front cone: '
            f'+/- {self.lidar_half_angle_deg:.1f} deg'
        )

        self.get_logger().info(
            f'Arm pick pose: '
            f'{self.arm_pick_positions}'
        )

        self.get_logger().info(
            '=============================================='
        )

    # =============================================================
    # CAMERA INFO
    # =============================================================

    def camera_info_callback(self, msg):

        if self.camera_matrix is not None:
            return

        try:

            self.camera_matrix = np.array(
                msg.k,
                dtype=np.float64
            ).reshape(3, 3)

            self.dist_coeffs = np.array(
                msg.d,
                dtype=np.float64
            )

            self.get_logger().info(
                'Camera calibration received'
            )

        except Exception as error:

            self.get_logger().error(
                f'Camera calibration error: {error}'
            )

    # =============================================================
    # MARKER ID CALLBACK
    # =============================================================

    def marker_id_callback(self, msg):

        if self.target_id in msg.data:

            self.get_logger().debug(
                f'Target ID {self.target_id} present'
            )

    # =============================================================
    # LASER CALLBACK
    # =============================================================

    def scan_callback(self, msg):

        self.latest_scan = msg

    # =============================================================
    # FRONT LIDAR DISTANCE
    # =============================================================

    def front_obstacle_distance(self):

        if self.latest_scan is None:
            return float('inf')

        distances = []

        angle = self.latest_scan.angle_min

        half_angle = math.radians(
            self.lidar_half_angle_deg
        )

        for distance in self.latest_scan.ranges:

            if abs(angle) <= half_angle:

                if math.isfinite(distance):

                    if (
                        distance >=
                        self.latest_scan.range_min
                        and
                        distance <=
                        self.latest_scan.range_max
                    ):
                        distances.append(distance)

            angle += self.latest_scan.angle_increment

        if not distances:
            return float('inf')

        return min(distances)

    # =============================================================
    # CAMERA IMAGE
    # =============================================================

    def image_callback(self, msg):

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

        except Exception as error:

            self.get_logger().error(
                f'Image conversion error: {error}'
            )

            return

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        try:

            corners, ids, rejected = (
                cv2.aruco.detectMarkers(
                    gray,
                    self.dictionary,
                    parameters=self.parameters
                )
            )

        except Exception as error:

            self.get_logger().error(
                f'ArUco detection error: {error}'
            )

            return

        self.target_seen = False

        # ---------------------------------------------------------
        # Draw all detected markers.
        # ---------------------------------------------------------

        if ids is not None:

            cv2.aruco.drawDetectedMarkers(
                frame,
                corners,
                ids
            )

            ids_flat = ids.flatten()

            for index, marker_id in enumerate(ids_flat):

                # IMPORTANT:
                # Only ID 7 is the mission target.
                if int(marker_id) != self.target_id:
                    continue

                if (
                    self.camera_matrix is None
                    or self.dist_coeffs is None
                ):
                    continue

                try:

                    rvecs, tvecs, _ = (
                        cv2.aruco
                        .estimatePoseSingleMarkers(
                            [corners[index]],
                            self.marker_size,
                            self.camera_matrix,
                            self.dist_coeffs
                        )
                    )

                except Exception as error:

                    self.get_logger().error(
                        f'Pose estimation error: {error}'
                    )

                    continue

                tvec = tvecs[0][0]
                rvec = rvecs[0][0]

                # -------------------------------------------------
                # Camera coordinates
                #
                # X = left/right
                # Z = forward
                # -------------------------------------------------

                self.target_x = float(
                    tvec[0]
                )

                self.target_z = float(
                    tvec[2]
                )

                self.target_seen = True

                # -------------------------------------------------
                # Draw pose axes
                # -------------------------------------------------

                try:

                    cv2.drawFrameAxes(
                        frame,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvec,
                        tvec,
                        0.05
                    )

                except Exception:
                    pass

                # -------------------------------------------------
                # Debug text
                # -------------------------------------------------

                cv2.putText(
                    frame,
                    f'TARGET ID: {self.target_id}',
                    (25, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    frame,
                    (
                        f'X={self.target_x:.2f} '
                        f'Z={self.target_z:.2f} m'
                    ),
                    (25, 75),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2
                )

                break

        # ---------------------------------------------------------
        # Target not visible
        # ---------------------------------------------------------

        if not self.target_seen:

            self.target_x = None
            self.target_z = None

            cv2.putText(
                frame,
                (
                    f'TARGET ID '
                    f'{self.target_id} NOT FOUND'
                ),
                (25, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

        # ---------------------------------------------------------
        # Publish debug image.
        # ---------------------------------------------------------

        try:

            debug_msg = (
                self.bridge.cv2_to_imgmsg(
                    frame,
                    encoding='bgr8'
                )
            )

            self.debug_pub.publish(
                debug_msg
            )

        except Exception as error:

            self.get_logger().error(
                f'Debug image error: {error}'
            )

    # =============================================================
    # START MISSION
    # =============================================================

    def start_mission(self):

        self.start_timer.cancel()

        if self.state != 'WAITING':
            return

        self.state = 'NAVIGATING'

        self.get_logger().info(
            'Waiting for Nav2 action server...'
        )

        if not self.nav_client.wait_for_server(
            timeout_sec=10.0
        ):

            self.get_logger().error(
                'Nav2 action server unavailable'
            )

            self.state = 'ERROR'

            return

        self.get_logger().info(
            'Nav2 action server available'
        )

        self.send_pick_goal()

    # =============================================================
    # SEND PICK NAV2 GOAL
    # =============================================================

    def send_pick_goal(self):

        goal = NavigateToPose.Goal()

        pose = PoseStamped()

        pose.header.frame_id = 'map'

        pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        pose.pose.position.x = self.pick_x
        pose.pose.position.y = self.pick_y
        pose.pose.position.z = 0.0

        qz = math.sin(
            self.pick_yaw / 2.0
        )

        qw = math.cos(
            self.pick_yaw / 2.0
        )

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal.pose = pose

        self.get_logger().info(
            'Sending Nav2 pickup-station goal...'
        )

        future = (
            self.nav_client.send_goal_async(
                goal
            )
        )

        future.add_done_callback(
            self.pick_goal_response
        )

    # =============================================================
    # PICK NAV2 GOAL RESPONSE
    # =============================================================

    def pick_goal_response(self, future):

        try:

            goal_handle = future.result()

        except Exception as error:

            self.get_logger().error(
                f'Nav2 goal response error: {error}'
            )

            self.state = 'ERROR'
            return

        if not goal_handle.accepted:

            self.get_logger().error(
                'Nav2 pickup goal rejected'
            )

            self.state = 'ERROR'

            return

        self.nav_goal_handle = goal_handle

        self.get_logger().info(
            'Nav2 pickup goal accepted'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.pick_nav_result
        )

    # =============================================================
    # PICK NAV2 RESULT
    # =============================================================

    def pick_nav_result(self, future):

        try:

            result = future.result()

        except Exception as error:

            self.get_logger().error(
                f'Nav2 result error: {error}'
            )

            self.state = 'ERROR'
            return

        if (
            result.status
            != GoalStatus.STATUS_SUCCEEDED
        ):

            self.get_logger().error(
                f'Nav2 pickup navigation failed. '
                f'Status={result.status}'
            )

            self.state = 'ERROR'

            return

        self.get_logger().info(
            '=========================================='
        )

        self.get_logger().info(
            'NAV2 PICKUP STATION REACHED'
        )

        self.get_logger().info(
            'Starting ArUco final approach'
        )

        self.get_logger().info(
            '=========================================='
        )

        self.state = 'VISUAL_APPROACH'

    # =============================================================
    # MAIN CONTROL LOOP
    # =============================================================

    def control_loop(self):

        # ---------------------------------------------------------
        # NAV2 phase
        #
        # IMPORTANT:
        # Even if ID 7 is visible during Nav2 navigation,
        # DO NOT cancel Nav2.
        # ---------------------------------------------------------

        if self.state == 'NAVIGATING':

            return

        # ---------------------------------------------------------
        # Visual phase
        # ---------------------------------------------------------

        if self.state in [
            'VISUAL_APPROACH',
            'ALIGNING',
            'APPROACHING'
        ]:

            self.visual_approach()

    # =============================================================
    # VISUAL APPROACH
    # =============================================================

    def visual_approach(self):

        # ---------------------------------------------------------
        # Marker lost
        # ---------------------------------------------------------

        if not self.target_seen:

            self.stop_robot()

            self.get_logger().debug(
                'ID 7 lost - robot stopped'
            )

            return

        x = self.target_x
        z = self.target_z

        # ---------------------------------------------------------
        # LIDAR SAFETY
        # ---------------------------------------------------------

        front_distance = (
            self.front_obstacle_distance()
        )

        if front_distance < self.min_lidar_distance:

            self.safety_stop_count += 1

            self.stop_robot()

            self.get_logger().warn(
                f'LiDAR warning: '
                f'{front_distance:.3f} m '
                f'('
                f'{self.safety_stop_count}/'
                f'{self.safety_required_count}'
                f')'
            )

            # Only enter permanent safety state after
            # consecutive unsafe readings.
            if (
                self.safety_stop_count
                >= self.safety_required_count
            ):

                self.state = 'SAFETY_STOP'

                self.get_logger().error(
                    '=========================================='
                )

                self.get_logger().error(
                    f'LASER SAFETY STOP: '
                    f'{front_distance:.3f} m'
                )

                self.get_logger().error(
                    'Robot stopped for safety.'
                )

                self.get_logger().error(
                    '=========================================='

                )

            return

        # ---------------------------------------------------------
        # Safe laser reading resets warning counter.
        # ---------------------------------------------------------

        self.safety_stop_count = 0

        # ---------------------------------------------------------
        # FINAL DISTANCE
        # ---------------------------------------------------------

        if z <= self.final_target_z:

            self.stop_robot()

            self.get_logger().info(
                '=========================================='
            )

            self.get_logger().info(
                'FINAL ARUCO POSITION REACHED'
            )

            self.get_logger().info(
                f'X = {x:.3f} m'
            )

            self.get_logger().info(
                f'Z = {z:.3f} m'
            )

            self.get_logger().info(
                'Mobile base STOPPED'
            )

            self.get_logger().info(
                'Starting PICK sequence'
            )

            self.get_logger().info(
                '=========================================='
            )

            # Marker no longer controls movement.
            self.target_seen = False

            self.state = 'OPENING_GRIPPER'

            self.open_gripper()

            return

        # ---------------------------------------------------------
        # TARGET NOT CENTERED
        # ---------------------------------------------------------

        if abs(x) > self.x_tolerance:

            self.state = 'ALIGNING'

            command = Twist()

            command.linear.x = 0.0

            # Camera:
            #
            # x < 0 = target is LEFT
            # x > 0 = target is RIGHT
            #
            # ROS:
            #
            # + angular.z = LEFT
            # - angular.z = RIGHT
            #
            # Therefore:
            #
            # angular.z = -K*x

            angular = -1.5 * x

            angular = max(
                -self.max_angular_speed,
                min(
                    self.max_angular_speed,
                    angular
                )
            )

            command.angular.z = angular

            self.cmd_vel_pub.publish(
                command
            )

            return

        # ---------------------------------------------------------
        # TARGET CENTERED
        # ---------------------------------------------------------

        self.state = 'APPROACHING'

        command = Twist()

        command.linear.x = self.approach_speed
        command.angular.z = 0.0

        self.cmd_vel_pub.publish(
            command
        )

    # =============================================================
    # STOP ROBOT
    # =============================================================

    def stop_robot(self):

        command = Twist()

        command.linear.x = 0.0
        command.angular.z = 0.0

        self.cmd_vel_pub.publish(
            command
        )

    # =============================================================
    # OPEN GRIPPER
    # =============================================================

    def open_gripper(self):

        self.stop_robot()

        self.state = 'OPENING_GRIPPER'

        self.get_logger().info(
            '=========================================='
        )

        self.get_logger().info(
            'OPENING GRIPPER'
        )

        self.get_logger().info(
            '=========================================='
        )

        if not self.gripper_client.wait_for_server(
            timeout_sec=10.0
        ):

            self.get_logger().error(
                'GRIPPER ACTION SERVER NOT AVAILABLE'
            )

            self.state = 'ERROR'

            return

        self.get_logger().info(
            'Gripper action server available'
        )

        goal = GripperCommand.Goal()

        # Tested simulation opening position.
        goal.command.position = 0.019
        goal.command.max_effort = 0.5

        self.get_logger().info(
            'Sending gripper OPEN command'
        )

        future = (
            self.gripper_client.send_goal_async(
                goal
            )
        )

        future.add_done_callback(
            self.open_gripper_response
        )

    # =============================================================
    # OPEN GRIPPER RESPONSE
    # =============================================================

    def open_gripper_response(self, future):

        try:

            goal_handle = future.result()

        except Exception as error:

            self.get_logger().error(
                f'Gripper open goal error: {error}'
            )

            self.state = 'ERROR'

            return

        if not goal_handle.accepted:

            self.get_logger().error(
                'GRIPPER OPEN GOAL REJECTED'
            )

            self.state = 'ERROR'

            return

        self.get_logger().info(
            'GRIPPER OPEN GOAL ACCEPTED'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.open_gripper_result
        )

    # =============================================================
    # OPEN GRIPPER RESULT
    # =============================================================

    def open_gripper_result(self, future):

        try:

            result = future.result()

        except Exception as error:

            self.get_logger().error(
                f'Gripper open result error: {error}'
            )

            self.state = 'ERROR'

            return

        self.get_logger().info(
            'GRIPPER OPEN SUCCESS'
        )

        self.state = 'MOVING_ARM'

        self.move_arm_to_pick()

    # =============================================================
    # ARM PICK POSITION
    # =============================================================

    def move_arm_to_pick(self):

        self.get_logger().info(
            'Moving arm to calibrated pick pose...'
        )

        if not self.arm_client.wait_for_server(
            timeout_sec=10.0
        ):

            self.get_logger().error(
                'ARM ACTION SERVER NOT AVAILABLE'
            )

            self.state = 'ERROR'

            return

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
        ]

        point = JointTrajectoryPoint()

        point.positions = list(
            self.arm_pick_positions
        )

        point.time_from_start.sec = 4

        goal.trajectory.points = [
            point
        ]

        future = (
            self.arm_client.send_goal_async(
                goal
            )
        )

        future.add_done_callback(
            self.arm_goal_response
        )

    # =============================================================
    # ARM RESPONSE
    # =============================================================

    def arm_goal_response(self, future):

        try:

            goal_handle = future.result()

        except Exception as error:

            self.get_logger().error(
                f'Arm goal response error: {error}'
            )

            self.state = 'ERROR'

            return

        if not goal_handle.accepted:

            self.get_logger().error(
                'ARM GOAL REJECTED'
            )

            self.state = 'ERROR'

            return

        self.get_logger().info(
            'ARM PICK GOAL ACCEPTED'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.arm_result
        )

    # =============================================================
    # ARM RESULT
    # =============================================================

    def arm_result(self, future):

        try:

            result = future.result()

        except Exception as error:

            self.get_logger().error(
                f'Arm result error: {error}'
            )

            self.state = 'ERROR'

            return

        if (
            result.status
            != GoalStatus.STATUS_SUCCEEDED
        ):

            self.get_logger().error(
                'ARM FAILED TO REACH PICK POSE'
            )

            self.state = 'ERROR'

            return

        self.get_logger().info(
            'ARM REACHED PICK POSE'
        )

        self.state = 'CLOSING_GRIPPER'

        self.close_gripper()

    # =============================================================
    # CLOSE GRIPPER
    # =============================================================

    def close_gripper(self):

        self.get_logger().info(
            '=========================================='
        )

        self.get_logger().info(
            'CLOSING GRIPPER'
        )

        self.get_logger().info(
            '=========================================='
        )

        if not self.gripper_client.wait_for_server(
            timeout_sec=10.0
        ):

            self.get_logger().error(
                'GRIPPER ACTION SERVER NOT AVAILABLE'
            )

            self.state = 'ERROR'

            return

        goal = GripperCommand.Goal()

        # Tested simulation closing position.
        goal.command.position = 0.0
        goal.command.max_effort = 0.8

        future = (
            self.gripper_client.send_goal_async(
                goal
            )
        )

        future.add_done_callback(
            self.close_gripper_response
        )

    # =============================================================
    # CLOSE GRIPPER RESPONSE
    # =============================================================

    def close_gripper_response(self, future):

        try:

            goal_handle = future.result()

        except Exception as error:

            self.get_logger().error(
                f'Close gripper goal error: {error}'
            )

            self.state = 'ERROR'

            return

        if not goal_handle.accepted:

            self.get_logger().error(
                'CLOSE GRIPPER GOAL REJECTED'
            )

            self.state = 'ERROR'

            return

        self.get_logger().info(
            'CLOSE GRIPPER GOAL ACCEPTED'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.close_gripper_result
        )

    # =============================================================
    # CLOSE GRIPPER RESULT
    # =============================================================

    def close_gripper_result(self, future):

        try:

            result = future.result()

        except Exception as error:

            self.get_logger().error(
                f'Close gripper result error: {error}'
            )

            self.state = 'ERROR'

            return

        self.get_logger().info(
            'GRIPPER CLOSED'
        )

        self.state = 'LIFTING'

        self.lift_arm()

    # =============================================================
    # LIFT ARM
    # =============================================================

    def lift_arm(self):

        self.get_logger().info(
            '=========================================='
        )

        self.get_logger().info(
            'LIFTING OBJECT'
        )

        self.get_logger().info(
            '=========================================='
        )

        if not self.arm_client.wait_for_server(
            timeout_sec=10.0
        ):

            self.get_logger().error(
                'ARM ACTION SERVER NOT AVAILABLE'
            )

            self.state = 'ERROR'

            return

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
        ]

        point = JointTrajectoryPoint()

        # Conservative lift configuration.
        point.positions = [
            0.0,
            0.70,
            -0.70,
            0.0,
        ]

        point.time_from_start.sec = 3

        goal.trajectory.points = [
            point
        ]

        future = (
            self.arm_client.send_goal_async(
                goal
            )
        )

        future.add_done_callback(
            self.lift_goal_response
        )

    # =============================================================
    # LIFT RESPONSE
    # =============================================================

    def lift_goal_response(self, future):

        try:

            goal_handle = future.result()

        except Exception as error:

            self.get_logger().error(
                f'Lift goal response error: {error}'
            )

            self.state = 'ERROR'

            return

        if not goal_handle.accepted:

            self.get_logger().error(
                'LIFT GOAL REJECTED'
            )

            self.state = 'ERROR'

            return

        self.get_logger().info(
            'LIFT GOAL ACCEPTED'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.lift_result
        )

    # =============================================================
    # LIFT RESULT
    # =============================================================

    def lift_result(self, future):

        try:

            result = future.result()

        except Exception as error:

            self.get_logger().error(
                f'Lift result error: {error}'
            )

            self.state = 'ERROR'

            return

        if (
            result.status
            != GoalStatus.STATUS_SUCCEEDED
        ):

            self.get_logger().error(
                'ARM LIFT FAILED'
            )

            self.state = 'ERROR'

            return

        self.get_logger().info(
            'ARM LIFT SUCCESS'
        )

        self.get_logger().info(
            'OBJECT SHOULD NOW BE HELD'
        )

        # Now send the mobile base home.
        self.send_home_goal()

    # =============================================================
    # RETURN HOME
    # =============================================================

    def send_home_goal(self):

        self.stop_robot()

        self.state = 'RETURNING_HOME'

        self.get_logger().info(
            '=========================================='
        )

        self.get_logger().info(
            'RETURNING HOME'
        )

        self.get_logger().info(
            f'Home X   = {self.home_x:.3f}'
        )

        self.get_logger().info(
            f'Home Y   = {self.home_y:.3f}'
        )

        self.get_logger().info(
            f'Home Yaw = {self.home_yaw:.3f}'
        )

        self.get_logger().info(
            '=========================================='
        )

        if not self.nav_client.wait_for_server(
            timeout_sec=10.0
        ):

            self.get_logger().error(
                'NAV2 ACTION SERVER UNAVAILABLE FOR HOME'
            )

            self.state = 'ERROR'

            return

        goal = NavigateToPose.Goal()

        pose = PoseStamped()

        pose.header.frame_id = 'map'

        pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        pose.pose.position.x = self.home_x
        pose.pose.position.y = self.home_y
        pose.pose.position.z = 0.0

        qz = math.sin(
            self.home_yaw / 2.0
        )

        qw = math.cos(
            self.home_yaw / 2.0
        )

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal.pose = pose

        self.get_logger().info(
            'Sending HOME goal to Nav2...'
        )

        future = (
            self.nav_client.send_goal_async(
                goal
            )
        )

        future.add_done_callback(
            self.home_goal_response
        )

    # =============================================================
    # HOME RESPONSE
    # =============================================================

    def home_goal_response(self, future):

        try:

            goal_handle = future.result()

        except Exception as error:

            self.get_logger().error(
                f'Home goal response error: {error}'
            )

            self.state = 'ERROR'

            return

        if not goal_handle.accepted:

            self.get_logger().error(
                'HOME GOAL REJECTED'
            )

            self.state = 'ERROR'

            return

        self.get_logger().info(
            'HOME GOAL ACCEPTED'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.home_result
        )

    # =============================================================
    # HOME RESULT
    # =============================================================

    def home_result(self, future):

        try:

            result = future.result()

        except Exception as error:

            self.get_logger().error(
                f'Home result error: {error}'
            )

            self.state = 'ERROR'

            return

        if (
            result.status
            != GoalStatus.STATUS_SUCCEEDED
        ):

            self.get_logger().error(
                f'HOME NAVIGATION FAILED: '
                f'status={result.status}'
            )

            self.state = 'ERROR'

            return

        self.stop_robot()

        self.state = 'MISSION_COMPLETE'

        self.get_logger().info(
            '=========================================='
        )

        self.get_logger().info(
            '       AUTONOMOUS MISSION COMPLETE'
        )

        self.get_logger().info(
            '       PICK + LIFT + RETURN HOME'
        )

        self.get_logger().info(
            '=========================================='

        )

    # =============================================================
    # CLEAN SHUTDOWN
    # =============================================================

    def destroy_node(self):

        try:
            self.stop_robot()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:

        node = PickMission()

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    except Exception as error:

        if node is not None:

            node.get_logger().error(
                f'Fatal mission error: {error}'
            )

        else:

            print(
                f'Fatal mission error: {error}'
            )

    finally:

        if node is not None:

            try:
                node.stop_robot()
            except Exception:
                pass

            try:
                node.destroy_node()
            except Exception:
                pass

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':
    main()