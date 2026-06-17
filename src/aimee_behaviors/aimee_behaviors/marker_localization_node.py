#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (C) ARDUINO SRL (http://www.arduino.cc)
#
# SPDX-License-Identifier: MPL-2.0

"""
ArUco marker localization node for the AIMEE Robot.

Subscribes to a camera image stream, detects ArUco markers, and publishes the
pose of each visible marker in the camera frame on /behavior/marker_poses/<id>.

The marker is assumed to lie flat on the operating surface (floor/table). The
resulting PoseStamped has the marker's Z axis pointing out of the surface.
"""

import math
from typing import List, Optional, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class MarkerLocalizationNode(Node):
    """Detect ArUco markers and publish their camera-frame poses."""

    def __init__(self):
        super().__init__("marker_localization_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("enabled", True),
                ("image_topic", "/camera/image_raw"),
                ("camera_info_topic", "/camera/camera_info"),
                ("marker_size_m", 0.05),
                ("marker_dictionary", "DICT_4X4_50"),
                ("publish_debug_image", False),
                ("debug_image_topic", "/behavior/marker_debug_image"),
                ("detection_rate_hz", 10.0),
            ],
        )

        self._enabled = self.get_parameter("enabled").value
        self._image_topic = self.get_parameter("image_topic").value
        self._camera_info_topic = self.get_parameter("camera_info_topic").value
        self._marker_size = max(0.01, self.get_parameter("marker_size_m").value)
        dict_name = self.get_parameter("marker_dictionary").value
        self._publish_debug = self.get_parameter("publish_debug_image").value
        self._debug_topic = self.get_parameter("debug_image_topic").value
        self._detection_rate = max(1.0, self.get_parameter("detection_rate_hz").value)

        # ArUco dictionary
        self._aruco_dict = self._get_aruco_dict(dict_name)
        if self._aruco_dict is None:
            self.get_logger().error(f"Unknown ArUco dictionary: {dict_name}")
            raise RuntimeError(f"Unknown ArUco dictionary: {dict_name}")

        # Detector - handle OpenCV 4.7+ API change
        try:
            self._detector_params = cv2.aruco.DetectorParameters()
            self._detector = cv2.aruco.ArucoDetector(self._aruco_dict, self._detector_params)
            self._use_detector = True
        except AttributeError:
            self._detector_params = cv2.aruco.DetectorParameters_create()
            self._detector = None
            self._use_detector = False

        # 3D corner coordinates of a marker centered at origin, lying in XY plane
        s = self._marker_size / 2.0
        self._marker_obj_points = np.array(
            [[-s, s, 0.0], [s, s, 0.0], [s, -s, 0.0], [-s, -s, 0.0]],
            dtype=np.float32,
        )

        self._bridge = CvBridge()
        self._camera_matrix: Optional[np.ndarray] = None
        self._dist_coeffs: Optional[np.ndarray] = None
        self._camera_info_valid = False

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscribers
        self.create_subscription(Image, self._image_topic, self._on_image, 10)
        self.create_subscription(CameraInfo, self._camera_info_topic, self._on_camera_info, 10)

        # Publishers
        self._visible_pub = self.create_publisher(String, "/behavior/visible_markers", reliable_qos)
        if self._publish_debug:
            self._debug_pub = self.create_publisher(Image, self._debug_topic, 10)

        # Dynamic pose publishers created as markers are seen
        self._pose_pubs: dict = {}

        # Rate limiting
        self._last_detection_time = 0.0

        self.get_logger().info(
            "MarkerLocalizationNode initialized:\n"
            f"  enabled: {self._enabled}\n"
            f"  image_topic: {self._image_topic}\n"
            f"  marker_size: {self._marker_size}m\n"
            f"  dictionary: {dict_name}"
        )

    def _get_aruco_dict(self, name: str):
        """Resolve ArUco dictionary name to OpenCV object."""
        attr = getattr(cv2.aruco, name, None)
        if attr is None:
            return None
        try:
            return cv2.aruco.getPredefinedDictionary(attr)
        except AttributeError:
            return cv2.aruco.Dictionary_get(attr)

    def _on_camera_info(self, msg: CameraInfo):
        if self._camera_info_valid:
            return
        self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        self._dist_coeffs = np.array(msg.d, dtype=np.float64)
        if self._dist_coeffs.size == 0:
            self._dist_coeffs = np.zeros((5,), dtype=np.float64)
        self._camera_info_valid = True
        self.get_logger().info("Camera info received; marker pose estimation ready")

    def _on_image(self, msg: Image):
        if not self._enabled:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_detection_time < (1.0 / self._detection_rate):
            return
        self._last_detection_time = now

        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warning(f"CV bridge error: {e}")
            return

        if not self._camera_info_valid:
            return

        markers = self._detect_markers(cv_image)
        self._publish_visible_markers([m[0] for m in markers])

        for marker_id, rvec, tvec in markers:
            self._publish_marker_pose(marker_id, rvec, tvec, msg.header.stamp)

        if self._publish_debug and markers:
            debug_image = self._draw_markers(cv_image, markers)
            try:
                debug_msg = self._bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
                debug_msg.header = msg.header
                self._debug_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().warning(f"Debug image publish error: {e}")

    def _detect_markers(
        self, image: np.ndarray
    ) -> List[Tuple[int, np.ndarray, np.ndarray]]:
        """Detect markers and estimate their camera-frame poses."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        if self._use_detector:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self._aruco_dict, parameters=self._detector_params
            )

        if ids is None or len(ids) == 0:
            return []

        results = []
        for i, corner in enumerate(corners):
            marker_id = int(ids[i][0])
            success, rvec, tvec = cv2.solvePnP(
                self._marker_obj_points,
                corner.reshape((4, 1, 2)).astype(np.float32),
                self._camera_matrix,
                self._dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if success:
                results.append((marker_id, rvec, tvec))
        return results

    def _rvec_to_quaternion(self, rvec: np.ndarray) -> Tuple[float, float, float, float]:
        """Convert Rodrigues rotation vector to quaternion (x, y, z, w)."""
        angle = np.linalg.norm(rvec)
        if angle < 1e-6:
            return (0.0, 0.0, 0.0, 1.0)
        axis = rvec.flatten() / angle
        half = angle / 2.0
        s = math.sin(half)
        c = math.cos(half)
        x = float(axis[0] * s)
        y = float(axis[1] * s)
        z = float(axis[2] * s)
        w = float(c)
        return (x, y, z, w)

    def _publish_marker_pose(self, marker_id: int, rvec: np.ndarray, tvec: np.ndarray, stamp):
        topic = f"/behavior/marker_poses/marker_{marker_id}"
        if topic not in self._pose_pubs:
            self._pose_pubs[topic] = self.create_publisher(PoseStamped, topic, 10)
            self.get_logger().info(f"Publishing marker pose on {topic}")

        x, y, z, w = self._rvec_to_quaternion(rvec)
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera"
        msg.pose.position.x = float(tvec[0])
        msg.pose.position.y = float(tvec[1])
        msg.pose.position.z = float(tvec[2])
        msg.pose.orientation.x = x
        msg.pose.orientation.y = y
        msg.pose.orientation.z = z
        msg.pose.orientation.w = w
        self._pose_pubs[topic].publish(msg)

    def _publish_visible_markers(self, marker_ids: List[int]):
        msg = String()
        msg.data = ",".join(str(mid) for mid in marker_ids) if marker_ids else ""
        self._visible_pub.publish(msg)

    def _draw_markers(
        self, image: np.ndarray, markers: List[Tuple[int, np.ndarray, np.ndarray]]
    ) -> np.ndarray:
        """Draw detected marker axes on the image for debugging."""
        out = image.copy()
        for marker_id, rvec, tvec in markers:
            try:
                cv2.drawFrameAxes(
                    out,
                    self._camera_matrix,
                    self._dist_coeffs,
                    rvec,
                    tvec,
                    self._marker_size / 2.0,
                )
            except Exception:
                pass
            # Also draw ID text near center if possible
            text = f"ID:{marker_id}"
            try:
                # Project origin of marker to image for label placement
                proj, _ = cv2.projectPoints(
                    np.zeros((1, 3)), rvec, tvec, self._camera_matrix, self._dist_coeffs
                )
                px, py = int(proj[0][0][0]), int(proj[0][0][1])
                cv2.putText(out, text, (px, py), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            except Exception:
                pass
        return out

    def destroy_node(self):
        self.get_logger().info("Shutting down MarkerLocalizationNode...")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MarkerLocalizationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
