#!/usr/bin/env python3

import math
import cv2
import numpy as np
import rclpy

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def get_aruco_dictionary(name):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("OpenCV aruco module is not available. Install opencv-contrib-python or ros-humble-vision-opencv with aruco support.")
    if not hasattr(cv2.aruco, name):
        raise RuntimeError(f"Unknown ArUco dictionary: {name}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def make_detector_parameters():
    if hasattr(cv2.aruco, "DetectorParameters"):
        return cv2.aruco.DetectorParameters()
    return cv2.aruco.DetectorParameters_create()


def rotation_matrix_to_quaternion(r):
    q = np.empty((4,), dtype=np.float64)
    trace = np.trace(r)

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q[3] = 0.25 * s
        q[0] = (r[2, 1] - r[1, 2]) / s
        q[1] = (r[0, 2] - r[2, 0]) / s
        q[2] = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        q[3] = (r[2, 1] - r[1, 2]) / s
        q[0] = 0.25 * s
        q[1] = (r[0, 1] + r[1, 0]) / s
        q[2] = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        q[3] = (r[0, 2] - r[2, 0]) / s
        q[0] = (r[0, 1] + r[1, 0]) / s
        q[1] = 0.25 * s
        q[2] = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        q[3] = (r[1, 0] - r[0, 1]) / s
        q[0] = (r[0, 2] + r[2, 0]) / s
        q[1] = (r[1, 2] + r[2, 1]) / s
        q[2] = 0.25 * s

    return q  # x, y, z, w


class ArucoEefTracker(Node):
    def __init__(self):
        super().__init__("aruco_eef_tracker_node")

        self.image_topic = self.declare_parameter("image_topic", "/eef_camera/image_raw").value
        self.camera_info_topic = self.declare_parameter("camera_info_topic", "/eef_camera/camera_info").value
        self.pose_topic = self.declare_parameter("pose_topic", "/target/aruco_pose").value
        self.visible_topic = self.declare_parameter("visible_topic", "/target/aruco_visible").value
        self.status_topic = self.declare_parameter("status_topic", "/target/aruco_status").value
        self.debug_image_topic = self.declare_parameter(
            "debug_image_topic", "/target/aruco_debug_image").value

        self.marker_id = int(self.declare_parameter("marker_id", 0).value)
        self.accept_any_marker = bool(
            self.declare_parameter("accept_any_marker", False).value)
        self.marker_size_m = float(self.declare_parameter("marker_size_m", 0.05).value)
        self.dictionary_name = str(
            self.declare_parameter("dictionary", "DICT_4X4_50").value)
        self.publish_debug_image = bool(
            self.declare_parameter("publish_debug_image", True).value)
        self.draw_axes = bool(self.declare_parameter("draw_axes", True).value)
        self.axis_length_m = float(self.declare_parameter("axis_length_m", 0.03).value)
        self.status_period_s = float(self.declare_parameter("status_period_s", 0.5).value)
        self.min_marker_perimeter_px = float(
            self.declare_parameter("min_marker_perimeter_px", 20.0).value)
        self.max_reprojection_error_px = float(
            self.declare_parameter("max_reprojection_error_px", 6.0).value)
        self.pose_z_min_m = float(self.declare_parameter("pose_z_min_m", 0.02).value)
        self.pose_z_max_m = float(self.declare_parameter("pose_z_max_m", 1.20).value)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.frame_id = "eef_usb_camera_optical_frame"
        self.last_status_text = None
        self.last_status_time = self.get_clock().now()

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.visible_pub = self.create_publisher(Bool, self.visible_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.on_image, qos_profile_sensor_data)
        self.info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.on_camera_info, qos_profile_sensor_data)

        self.aruco_dict = get_aruco_dictionary(self.dictionary_name)
        self.aruco_params = make_detector_parameters()
        if hasattr(self.aruco_params, "minMarkerPerimeterRate"):
            self.aruco_params.minMarkerPerimeterRate = 0.01

        self.detector = None
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.publish_status(
            f"aruco_eef_tracker ready: image={self.image_topic}, camera_info={self.camera_info_topic}, marker_id={self.marker_id}, marker_size={self.marker_size_m}")

    def publish_status(self, text):
        now = self.get_clock().now()
        if (
            text == self.last_status_text and
            (now - self.last_status_time).nanoseconds * 1.0e-9 < self.status_period_s
        ):
            return
        self.last_status_text = text
        self.last_status_time = now
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def publish_visible(self, visible):
        msg = Bool()
        msg.data = bool(visible)
        self.visible_pub.publish(msg)

    def on_camera_info(self, msg):
        camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        if not np.isfinite(camera_matrix).all() or camera_matrix[0, 0] <= 0.0 or camera_matrix[1, 1] <= 0.0:
            self.publish_status("invalid camera_info; waiting for calibrated EEF camera_info")
            return
        self.camera_matrix = camera_matrix
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        if len(self.dist_coeffs) == 0:
            self.dist_coeffs = np.zeros((5,), dtype=np.float64)
        self.frame_id = msg.header.frame_id or self.frame_id

    def detect_markers(self, gray):
        if self.detector is not None:
            return self.detector.detectMarkers(gray)
        return cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params)

    def publish_debug(self, frame, stamp, text=None, corners=None, ids=None, rvec=None, tvec=None):
        if not self.publish_debug_image:
            return
        if corners is not None and ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        if (
            self.draw_axes and rvec is not None and tvec is not None and
            self.camera_matrix is not None and hasattr(cv2, "drawFrameAxes")
        ):
            cv2.drawFrameAxes(
                frame,
                self.camera_matrix,
                self.dist_coeffs,
                rvec,
                tvec,
                self.axis_length_m)
        if text:
            cv2.putText(
                frame,
                text,
                (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
                cv2.LINE_AA)
        debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        debug_msg.header.stamp = stamp
        debug_msg.header.frame_id = self.frame_id
        self.debug_pub.publish(debug_msg)

    def marker_perimeter(self, corner):
        pts = corner.reshape((4, 2))
        return float(sum(np.linalg.norm(pts[(i + 1) % 4] - pts[i]) for i in range(4)))

    def select_marker(self, corners, ids):
        ids_flat = ids.flatten().tolist()
        candidates = []
        for index, marker in enumerate(ids_flat):
            if self.accept_any_marker or marker == self.marker_id:
                perimeter = self.marker_perimeter(corners[index])
                if perimeter >= self.min_marker_perimeter_px:
                    candidates.append((perimeter, index, marker))
        if not candidates:
            return None, ids_flat
        candidates.sort(reverse=True)
        _, index, marker = candidates[0]
        return (index, marker), ids_flat

    def reprojection_error(self, object_points, image_points, rvec, tvec):
        projected, _ = cv2.projectPoints(
            object_points,
            rvec,
            tvec,
            self.camera_matrix,
            self.dist_coeffs)
        projected = projected.reshape((-1, 2))
        return float(np.mean(np.linalg.norm(projected - image_points, axis=1)))

    def on_image(self, msg):
        if self.camera_matrix is None:
            self.publish_visible(False)
            self.publish_status("waiting for camera_info")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as ex:
            self.publish_visible(False)
            self.publish_status(f"image conversion failed: {ex}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detect_markers(gray)

        if ids is None or len(ids) == 0:
            self.publish_visible(False)
            self.publish_debug(frame, msg.header.stamp, "aruco: not visible")
            self.publish_status("aruco marker not visible")
            return

        selected, ids_flat = self.select_marker(corners, ids)
        if selected is None:
            self.publish_visible(False)
            self.publish_debug(frame, msg.header.stamp, f"aruco: rejected ids={ids_flat}", corners, ids)
            self.publish_status(
                f"aruco marker id {self.marker_id} not usable; detected={ids_flat}, min_perimeter={self.min_marker_perimeter_px:.1f}px")
            return

        marker_index, selected_id = selected
        image_points = corners[marker_index].reshape((4, 2)).astype(np.float64)

        s = self.marker_size_m * 0.5
        object_points = np.array([
            [-s,  s, 0.0],
            [ s,  s, 0.0],
            [ s, -s, 0.0],
            [-s, -s, 0.0],
        ], dtype=np.float64)

        ok, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)

        if not ok:
            self.publish_visible(False)
            self.publish_debug(frame, msg.header.stamp, "aruco: solvePnP failed", corners, ids)
            self.publish_status("solvePnP failed")
            return

        pose_z = float(tvec[2][0])
        if pose_z < self.pose_z_min_m or pose_z > self.pose_z_max_m:
            self.publish_visible(False)
            self.publish_debug(frame, msg.header.stamp, f"aruco: z rejected {pose_z:.3f}m", corners, ids)
            self.publish_status(
                f"aruco pose rejected: z={pose_z:.3f}m outside [{self.pose_z_min_m:.3f}, {self.pose_z_max_m:.3f}]")
            return

        reproj_error = self.reprojection_error(object_points, image_points, rvec, tvec)
        if reproj_error > self.max_reprojection_error_px:
            self.publish_visible(False)
            self.publish_debug(frame, msg.header.stamp, f"aruco: reproj {reproj_error:.1f}px", corners, ids)
            self.publish_status(
                f"aruco pose rejected: reprojection_error={reproj_error:.2f}px > {self.max_reprojection_error_px:.2f}px")
            return

        rot, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = rotation_matrix_to_quaternion(rot)

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = float(tvec[0][0])
        pose.pose.position.y = float(tvec[1][0])
        pose.pose.position.z = float(tvec[2][0])
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)

        self.pose_pub.publish(pose)
        self.publish_visible(True)

        self.publish_status(
            f"aruco visible id={selected_id} xyz=({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, {pose.pose.position.z:.3f}) reproj={reproj_error:.2f}px frame={pose.header.frame_id}")

        self.publish_debug(
            frame,
            msg.header.stamp,
            f"id={selected_id} z={pose.pose.position.z:.3f}m err={reproj_error:.1f}px",
            corners,
            ids,
            rvec,
            tvec)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoEefTracker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
