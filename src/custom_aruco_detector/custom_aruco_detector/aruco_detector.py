import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32MultiArray

from cv_bridge import CvBridge

import cv2
import numpy as np


class CustomArucoDetector(Node):

    def __init__(self):

        super().__init__('custom_aruco_detector')

        # =====================================================
        # TARGET MARKER
        # =====================================================

        self.declare_parameter('target_id', 7)

        self.target_id = (
            self.get_parameter('target_id')
            .get_parameter_value()
            .integer_value
        )

        # =====================================================
        # ROS
        # =====================================================

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/pi_camera/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/pi_camera/camera_info',
            self.camera_info_callback,
            10
        )

        self.id_pub = self.create_publisher(
            Int32MultiArray,
            '/aruco/marker_ids',
            10
        )

        # ONLY TARGET POSE
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/aruco/target_pose',
            10
        )

        self.debug_pub = self.create_publisher(
            Image,
            '/aruco/debug_image',
            10
        )

        # =====================================================
        # ARUCO
        # =====================================================

        self.dictionary = cv2.aruco.Dictionary_get(
            cv2.aruco.DICT_5X5_50
        )

        self.parameters = (
            cv2.aruco.DetectorParameters_create()
        )

        # =====================================================
        # CAMERA
        # =====================================================

        self.camera_matrix = None
        self.dist_coeffs = None

        self.marker_size = 0.088

        self.get_logger().info(
            f'Target marker ID = {self.target_id}'
        )

    def camera_info_callback(self, msg):

        self.camera_matrix = np.array(
            msg.k,
            dtype=np.float64
        ).reshape(3, 3)

        self.dist_coeffs = np.array(
            msg.d,
            dtype=np.float64
        )

    def image_callback(self, msg):

        if self.camera_matrix is None:
            return

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            gray = cv2.cvtColor(
                frame,
                cv2.COLOR_BGR2GRAY
            )

            corners, ids, rejected = (
                cv2.aruco.detectMarkers(
                    gray,
                    self.dictionary,
                    parameters=self.parameters
                )
            )

            all_ids = []

            # =================================================
            # No marker
            # =================================================

            if ids is None:

                self.publish_empty()

                cv2.putText(
                    frame,
                    f'TARGET ID {self.target_id}: NOT FOUND',
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2
                )

                self.publish_debug(frame)

                return

            ids = ids.flatten()

            all_ids = ids.tolist()

            # Publish all detected IDs for debugging
            id_msg = Int32MultiArray()
            id_msg.data = all_ids

            self.id_pub.publish(id_msg)

            # Draw all detected markers
            cv2.aruco.drawDetectedMarkers(
                frame,
                corners,
                ids
            )

            # =================================================
            # SEARCH ONLY FOR TARGET ID
            # =================================================

            target_index = None

            for i, marker_id in enumerate(ids):

                if int(marker_id) == int(self.target_id):

                    target_index = i
                    break

            # =================================================
            # TARGET NOT FOUND
            # =================================================

            if target_index is None:

                cv2.putText(
                    frame,
                    f'TARGET {self.target_id} NOT FOUND',
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2
                )

                self.publish_debug(frame)

                return

            # =================================================
            # TARGET FOUND
            # =================================================

            target_corner = [
                corners[target_index]
            ]

            target_id_array = np.array(
                [[self.target_id]],
                dtype=np.int32
            )

            cv2.aruco.drawDetectedMarkers(
                frame,
                target_corner,
                target_id_array
            )

            # =================================================
            # POSE
            # =================================================

            rvecs, tvecs, _ = (
                cv2.aruco.estimatePoseSingleMarkers(
                    target_corner,
                    self.marker_size,
                    self.camera_matrix,
                    self.dist_coeffs
                )
            )

            rvec = rvecs[0][0]
            tvec = tvecs[0][0]

            x = float(tvec[0])
            y = float(tvec[1])
            z = float(tvec[2])

            # =================================================
            # DEBUG TEXT
            # =================================================

            center = (
                target_corner[0][0]
                .reshape(4, 2)
                .mean(axis=0)
            )

            center_x = int(center[0])
            center_y = int(center[1])

            cv2.circle(
                frame,
                (center_x, center_y),
                6,
                (0, 0, 255),
                -1
            )

            cv2.putText(
                frame,
                f'TARGET ID: {self.target_id}',
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 255, 0),
                2
            )

            cv2.putText(
                frame,
                f'X={x:.2f}  Z={z:.2f} m',
                (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2
            )

            # =================================================
            # POSE PUBLISH
            # =================================================

            pose_msg = PoseStamped()

            pose_msg.header = msg.header
            pose_msg.header.frame_id = 'base_footprint'

            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = z

            pose_msg.pose.orientation.w = 1.0

            self.pose_pub.publish(
                pose_msg
            )

            # =================================================
            # DRAW AXIS
            # =================================================

            cv2.aruco.drawAxis(
                frame,
                self.camera_matrix,
                self.dist_coeffs,
                rvec,
                tvec,
                0.05
            )

            self.publish_debug(frame)

        except Exception as e:

            self.get_logger().error(
                f'Processing error: {e}'
            )

    def publish_empty(self):

        msg = Int32MultiArray()
        msg.data = []

        self.id_pub.publish(msg)

    def publish_debug(self, frame):

        debug_msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding='bgr8'
        )

        self.debug_pub.publish(
            debug_msg
        )


def main(args=None):

    rclpy.init(args=args)

    node = CustomArucoDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()