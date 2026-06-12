#!/usr/bin/env python3

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Bool, String
import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped transform support
import tf2_ros


class ArucoToMpControlBridge(Node):
    def __init__(self):
        super().__init__("aruco_to_mp_control_bridge")

        self.aruco_pose_topic = self.declare_parameter(
            "aruco_pose_topic", "/target/aruco_pose").value
        self.aruco_visible_topic = self.declare_parameter(
            "aruco_visible_topic", "/target/aruco_visible").value
        self.object_topic = self.declare_parameter(
            "object_topic", "/target/object_in_base").value
        self.close_range_ready_topic = self.declare_parameter(
            "close_range_ready_topic", "/target/close_range_ready").value
        self.start_topic = self.declare_parameter("start_topic", "/mp_control/start").value
        self.status_topic = self.declare_parameter(
            "status_topic", "/target/aruco_mp_bridge_status").value

        self.target_frame = self.declare_parameter("target_frame", "base_link").value
        self.visible_timeout_s = float(self.declare_parameter("visible_timeout_s", 0.5).value)
        self.pose_timeout_s = float(self.declare_parameter("pose_timeout_s", 0.5).value)
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 20.0).value)
        self.force_object_x_m = float(self.declare_parameter("force_object_x_m", 0.22).value)
        self.object_y_offset_m = float(self.declare_parameter("object_y_offset_m", 0.0).value)
        self.object_z_offset_m = float(self.declare_parameter("object_z_offset_m", 0.0).value)
        self.publish_start_on_visible = bool(
            self.declare_parameter("publish_start_on_visible", True).value)
        self.start_publish_count = int(self.declare_parameter("start_publish_count", 3).value)
        self.start_publish_period_s = float(
            self.declare_parameter("start_publish_period_s", 0.2).value)
        self.continuous_start_publish = bool(
            self.declare_parameter("continuous_start_publish", True).value)
        self.publish_close_range_ready = bool(
            self.declare_parameter("publish_close_range_ready", True).value)
        self.locked_object_xyz = None
        self.start_sent_once = False

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.object_pub = self.create_publisher(PointStamped, self.object_topic, 10)
        self.ready_pub = self.create_publisher(Bool, self.close_range_ready_topic, latched_qos)
        self.start_pub = self.create_publisher(Bool, self.start_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, latched_qos)

        self.pose_sub = self.create_subscription(
            PoseStamped, self.aruco_pose_topic, self.on_pose, 10)
        self.visible_sub = self.create_subscription(
            Bool, self.aruco_visible_topic, self.on_visible, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_pose = None
        self.latest_pose_time = self.get_clock().now()
        self.visible = False
        self.latest_visible_time = self.get_clock().now()
        self.start_published = 0
        self.last_start_time = self.get_clock().now()
        self.last_status = ""

        period = 1.0 / max(1.0, self.publish_rate_hz)
        self.timer = self.create_timer(period, self.on_timer)
        self.publish_status(
            f"aruco bridge ready: pose={self.aruco_pose_topic}, object={self.object_topic}, forced_x={self.force_object_x_m:.3f}m")

    def on_pose(self, msg):
        self.latest_pose = msg
        self.latest_pose_time = self.get_clock().now()

    def on_visible(self, msg):
        self.visible = bool(msg.data)
        self.latest_visible_time = self.get_clock().now()
        if not self.visible:
            self.start_published = 0

    def fresh(self, stamp, timeout_s):
        return (self.get_clock().now() - stamp).nanoseconds * 1.0e-9 <= timeout_s

    def publish_status(self, text):
        if text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def publish_ready(self, ready):
        if not self.publish_close_range_ready:
            return
        msg = Bool()
        msg.data = bool(ready)
        self.ready_pub.publish(msg)

    def maybe_publish_start(self):
        if not self.publish_start_on_visible:
            return
        if not self.continuous_start_publish and self.start_published >= self.start_publish_count:
            return
        now = self.get_clock().now()
        if (
            self.start_published > 0 and
            (now - self.last_start_time).nanoseconds * 1.0e-9 < self.start_publish_period_s
        ):
            return
        msg = Bool()
        msg.data = True
        self.start_pub.publish(msg)
        self.start_published += 1
        if self.continuous_start_publish and self.start_publish_count > 0:
            self.start_published = min(self.start_published, self.start_publish_count)
        self.last_start_time = now

    def transformed_object_point(self):
        if self.latest_pose is None:
            return None, "waiting for aruco pose"
        if not self.fresh(self.latest_pose_time, self.pose_timeout_s):
            return None, "aruco pose stale"

        try:
            pose_msg = PoseStamped()
            pose_msg.header = self.latest_pose.header
            pose_msg.pose = self.latest_pose.pose

            pose_msg.header.stamp = rclpy.time.Time().to_msg()
            pose = self.tf_buffer.transform(
                pose_msg,
                self.target_frame,
                timeout=Duration(seconds=0.10))
        except Exception as exc:
            return None, f"TF {self.latest_pose.header.frame_id}->{self.target_frame} failed: {exc}"

        point = PointStamped()
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = self.target_frame
        point.point.x = float(pose.pose.position.x)
        point.point.y = float(pose.pose.position.y + self.object_y_offset_m)
        point.point.z = float(pose.pose.position.z + self.object_z_offset_m)

        if math.isfinite(self.force_object_x_m) and self.force_object_x_m >= 0.0:
            point.point.x = self.force_object_x_m

        return point, None

    def on_timer(self):
        visible_ready = self.visible and self.fresh(
            self.latest_visible_time, self.visible_timeout_s)
        if not visible_ready:
            self.publish_ready(False)
            self.publish_status("waiting for visible aruco marker")
            return

        point, reason = self.transformed_object_point()
        if point is None:
            self.publish_ready(False)
            self.publish_status(reason)
            return

        self.object_pub.publish(point)
        self.publish_ready(True)
        self.maybe_publish_start()
        self.publish_status(
            f"aruco object ready: xyz=({point.point.x:.3f}, {point.point.y:.3f}, {point.point.z:.3f}) frame={point.header.frame_id}")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoToMpControlBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
