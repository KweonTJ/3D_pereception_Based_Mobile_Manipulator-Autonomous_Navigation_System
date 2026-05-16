#!/usr/bin/env python3
"""Upload robot status and camera frames to the Raspberry Pi monitor server.

Run this on each robot. The node subscribes to local ROS 2 topics and sends
outbound HTTP uploads, so it does not need a ROS domain bridge.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
import urllib.error
import urllib.request
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Tuple

try:
    import cv2
    import numpy as np
except ImportError:  # Video can be disabled while keeping status uploads alive.
    cv2 = None
    np = None

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import String


def finite_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def yaw_from_quaternion(q: Any) -> Optional[float]:
    try:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return finite_float(math.atan2(siny_cosp, cosy_cosp))
    except Exception:
        return None


def battery_status_label(status: int) -> str:
    labels = {
        BatteryState.POWER_SUPPLY_STATUS_UNKNOWN: "unknown",
        BatteryState.POWER_SUPPLY_STATUS_CHARGING: "charging",
        BatteryState.POWER_SUPPLY_STATUS_DISCHARGING: "discharging",
        BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING: "not_charging",
        BatteryState.POWER_SUPPLY_STATUS_FULL: "full",
    }
    return labels.get(int(status), "unknown")


def battery_payload(msg: BatteryState) -> Dict[str, Any]:
    percentage = finite_float(msg.percentage)
    if percentage is not None and percentage <= 1.0:
        percentage *= 100.0
    return {
        "percentage": percentage,
        "voltage": finite_float(msg.voltage),
        "current": finite_float(msg.current),
        "charge": finite_float(msg.charge),
        "capacity": finite_float(msg.capacity),
        "status": battery_status_label(msg.power_supply_status),
    }


def twist_payload(msg: Twist) -> Dict[str, Optional[float]]:
    linear_x = finite_float(msg.linear.x)
    linear_y = finite_float(msg.linear.y)
    angular_z = finite_float(msg.angular.z)
    speed = math.hypot(float(linear_x or 0.0), float(linear_y or 0.0))
    return {
        "linear_x": linear_x,
        "linear_y": linear_y,
        "speed_mps": finite_float(speed),
        "angular_z": angular_z,
    }


def odom_payload(msg: Odometry) -> Dict[str, Any]:
    pose = msg.pose.pose
    payload = twist_payload(msg.twist.twist)
    payload.update(
        {
            "x": finite_float(pose.position.x),
            "y": finite_float(pose.position.y),
            "yaw": yaw_from_quaternion(pose.orientation),
        }
    )
    return payload


def image_to_bgr(msg: Image) -> Any:
    if cv2 is None or np is None:
        raise RuntimeError("OpenCV/numpy are required for video uploads")

    encoding = str(msg.encoding or "").lower()
    width = int(msg.width)
    height = int(msg.height)
    step = int(msg.step) if int(msg.step) else width

    if encoding in {"rgb8", "bgr8"}:
        row = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, step))
        image = row[:, : width * 3].reshape((height, width, 3))
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if encoding == "rgb8" else image.copy()

    if encoding in {"mono8", "8uc1"}:
        row = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, step))
        image = row[:, :width]
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    if encoding in {"16uc1", "mono16"}:
        row = np.frombuffer(msg.data, dtype=np.uint16).reshape((height, max(1, step // 2)))
        image = row[:, :width]
        valid = image[image > 0]
        if valid.size:
            near = np.percentile(valid, 5)
            far = np.percentile(valid, 95)
            if far <= near:
                far = near + 1.0
            scaled = np.clip((image.astype(np.float32) - near) * 255.0 / (far - near), 0, 255)
        else:
            scaled = np.zeros_like(image, dtype=np.float32)
        return cv2.applyColorMap(scaled.astype(np.uint8), cv2.COLORMAP_TURBO)

    if encoding == "32fc1":
        row = np.frombuffer(msg.data, dtype=np.float32).reshape((height, max(1, step // 4)))
        image = row[:, :width]
        finite = image[np.isfinite(image) & (image > 0)]
        if finite.size:
            near = np.percentile(finite, 5)
            far = np.percentile(finite, 95)
            if far <= near:
                far = near + 1.0
            scaled = np.clip((image - near) * 255.0 / (far - near), 0, 255)
        else:
            scaled = np.zeros_like(image, dtype=np.float32)
        return cv2.applyColorMap(scaled.astype(np.uint8), cv2.COLORMAP_TURBO)

    if encoding in {"yuv422", "yuyv", "yuv422_yuy2"}:
        row = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, step))
        image = row[:, : width * 2].reshape((height, width, 2))
        return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUY2)

    channels = max(1, int(step / width)) if width else 1
    if channels >= 3:
        row = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, step))
        image = row[:, : width * channels].reshape((height, width, channels))
        return image[:, :, :3].copy()

    row = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, step))
    return cv2.cvtColor(row[:, :width], cv2.COLOR_GRAY2BGR)


def resize_frame(frame: Any, width: int, height: int) -> Any:
    if width <= 0 and height <= 0:
        return frame
    h, w = frame.shape[:2]
    if width > 0 and height > 0:
        target = (width, height)
    elif width > 0:
        ratio = width / float(w)
        target = (width, max(1, int(h * ratio)))
    else:
        ratio = height / float(h)
        target = (max(1, int(w * ratio)), height)
    return cv2.resize(frame, target, interpolation=cv2.INTER_AREA)


class RobotStatusUploader(Node):
    def __init__(
        self,
        robot: str,
        server: str,
        token: str,
        status_period_s: float,
        video_period_s: float,
        jpeg_quality: int,
        video_enabled: bool,
        image_width: int,
        image_height: int,
        http_timeout_s: float,
    ) -> None:
        super().__init__(f"{robot}_upload_only_status_uploader")
        self.robot = robot
        self.server = server.rstrip("/")
        self.token = token
        self.status_period_s = status_period_s
        self.video_period_s = video_period_s
        self.jpeg_quality = max(1, min(95, int(jpeg_quality)))
        self.video_enabled = video_enabled
        self.image_width = image_width
        self.image_height = image_height
        self.http_timeout_s = http_timeout_s

        self.pending: Dict[str, Any] = {}
        self.event_queue: Deque[Any] = deque(maxlen=100)
        self.pending_lock = threading.RLock()
        self.last_video_sent: Dict[str, float] = {}
        self.video_busy: set[str] = set()
        self.frame_seen_counts: Dict[str, int] = {}
        self.image_subscriptions: List[Any] = []
        self.seq = 0
        self.last_status_error_log = 0.0

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        live_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        if self.robot == "leader":
            self._subscribe_leader(state_qos, live_qos)
        elif self.robot == "follower":
            self._subscribe_follower(state_qos, live_qos)
        else:
            raise ValueError("--robot must be leader or follower")

        self.create_timer(self.status_period_s, self.flush_status)
        self.get_logger().info(
            f"upload-only mode: robot={self.robot}, server={self.server}, "
            f"status_period={self.status_period_s}s, video_enabled={self.video_enabled}"
        )

    def _set_pending(self, key: str, value: Any) -> None:
        with self.pending_lock:
            self.pending[key] = value

    def _append_event(self, value: Any) -> None:
        with self.pending_lock:
            self.event_queue.append(value)

    def _subscribe_image_stream(
        self,
        stream_key: str,
        topics: List[str],
        live_qos: QoSProfile,
    ) -> None:
        unique_topics: List[str] = []
        for topic in topics:
            topic = str(topic).strip()
            if topic and topic not in unique_topics:
                unique_topics.append(topic)

        if not unique_topics:
            return

        self._set_pending(f"video.{stream_key}_topics", unique_topics)
        for topic in unique_topics:
            subscription = self.create_subscription(
                Image,
                topic,
                lambda msg, key=stream_key, source_topic=topic: self._image_callback(
                    key, msg, source_topic
                ),
                live_qos,
            )
            self.image_subscriptions.append(subscription)

    def _subscribe_leader(self, state_qos: QoSProfile, live_qos: QoSProfile) -> None:
        self.create_subscription(String, "/leader/task_state", lambda msg: self._set_pending("task_state", msg.data), state_qos)
        self.create_subscription(String, "/leader/cargo_state", lambda msg: self._set_pending("cargo_state", msg.data), state_qos)
        self.create_subscription(Bool, "/leader/follower_enable", lambda msg: self._set_pending("follower_enable", bool(msg.data)), state_qos)
        self.create_subscription(String, "/leader/platoon_mode", lambda msg: self._set_pending("platoon_mode", msg.data), state_qos)
        self.create_subscription(Bool, "/leader/heartbeat", lambda msg: self._set_pending("heartbeat", bool(msg.data)), live_qos)
        self.create_subscription(Twist, "/leader/cmd_vel", lambda msg: self._set_pending("cmd_vel", twist_payload(msg)), live_qos)
        self.create_subscription(Odometry, "/leader/odom", lambda msg: self._set_pending("odom", odom_payload(msg)), live_qos)
        self.create_subscription(BatteryState, "/battery_state", lambda msg: self._set_pending("battery", battery_payload(msg)), live_qos)
        self.create_subscription(BatteryState, "/leader/battery_state", lambda msg: self._set_pending("battery", battery_payload(msg)), live_qos)

        self.create_subscription(String, "/mp_control/status", lambda msg: self._set_pending("task.mp_control_status", msg.data), state_qos)
        self.create_subscription(String, "/mp_control/pick_place_status", lambda msg: self._set_pending("task.pick_place_status", msg.data), live_qos)
        self.create_subscription(String, "/cargo/current_id", lambda msg: self._set_pending("cargo.current_id", str(msg.data).strip()), state_qos)
        self.create_subscription(String, "/cargo/events", lambda msg: self._append_event(msg.data), live_qos)

        if self.video_enabled:
            self._subscribe_image_stream(
                "leader_debug",
                ["/hybrid_csrt_ibvs/debug_image"],
                live_qos,
            )
            self._subscribe_image_stream(
                "leader_raw",
                [
                    "/camera/color/image_raw",
                    "/camera/rgb/image_raw",
                    "/camera/image_raw",
                    "/camera/color/image",
                ],
                live_qos,
            )
            self._subscribe_image_stream(
                "leader_depth",
                ["/camera/depth/image_raw", "/camera/depth/image"],
                live_qos,
            )
            self._subscribe_image_stream(
                "eef_raw",
                ["/eef_camera/image_raw"],
                live_qos,
            )
            self._subscribe_image_stream(
                "eef_debug",
                ["/eef_hybrid_csrt_ibvs/debug_image"],
                live_qos,
            )

    def _subscribe_follower(self, state_qos: QoSProfile, live_qos: QoSProfile) -> None:
        self.create_subscription(String, "/follower/status", lambda msg: self._set_pending("status", msg.data), live_qos)
        self.create_subscription(String, "/follower/safety_state", lambda msg: self._set_pending("safety_state", msg.data), live_qos)
        self.create_subscription(Twist, "/cmd_vel", lambda msg: self._set_pending("cmd_vel", twist_payload(msg)), live_qos)
        self.create_subscription(Twist, "/follower/cmd_vel_raw", lambda msg: self._set_pending("cmd_vel_raw", twist_payload(msg)), live_qos)
        self.create_subscription(Odometry, "/odom", lambda msg: self._set_pending("odom", odom_payload(msg)), live_qos)
        self.create_subscription(BatteryState, "/battery_state", lambda msg: self._set_pending("battery", battery_payload(msg)), live_qos)
        self.create_subscription(BatteryState, "/follower/battery_state", lambda msg: self._set_pending("battery", battery_payload(msg)), live_qos)
        self.create_subscription(Float32, "/follower/distance_error", lambda msg: self._set_pending("distance_error_m", finite_float(msg.data)), live_qos)
        self.create_subscription(Bool, "/follower/target_visible", lambda msg: self._set_pending("target_visible", bool(msg.data)), live_qos)
        self.create_subscription(Float32, "/follower/target_distance", lambda msg: self._set_pending("target_distance_m", finite_float(msg.data)), live_qos)
        self.create_subscription(Float32, "/follower/target_offset_x", lambda msg: self._set_pending("target_offset_x", finite_float(msg.data)), live_qos)

        if self.video_enabled:
            self._subscribe_image_stream(
                "follower_raw",
                ["/follower/camera/image_raw", "/camera/image_raw", "/image_raw"],
                live_qos,
            )

    def _json_headers(self) -> Dict[str, str]:
        headers = {"Content-Type": "application/json"}
        if self.token:
            headers["X-Monitor-Token"] = self.token
        return headers

    def _jpeg_headers(self) -> Dict[str, str]:
        headers = {
            "Content-Type": "image/jpeg",
            "X-Monitor-Robot": self.robot,
            "X-Robot-Timestamp": str(time.time()),
        }
        if self.token:
            headers["X-Monitor-Token"] = self.token
        return headers

    def _post_json(self, path: str, payload: Dict[str, Any], timeout_s: Optional[float] = None) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        req = urllib.request.Request(self.server + path, data=body, headers=self._json_headers(), method="POST")
        with urllib.request.urlopen(req, timeout=timeout_s or self.http_timeout_s) as res:
            res.read()

    def _post_jpeg(self, path: str, jpeg: bytes, timeout_s: Optional[float] = None) -> None:
        req = urllib.request.Request(self.server + path, data=jpeg, headers=self._jpeg_headers(), method="POST")
        with urllib.request.urlopen(req, timeout=timeout_s or self.http_timeout_s) as res:
            res.read()

    def flush_status(self) -> None:
        self.seq += 1
        now = time.time()

        with self.pending_lock:
            payload_values = dict(self.pending)
            self.pending.clear()
            events = list(self.event_queue)
            self.event_queue.clear()

        payload_values.update(
            {
                "uploader_alive": True,
                "uploader_seq": self.seq,
                "uploader_time": now,
            }
        )

        payload = {
            "robot": self.robot,
            "timestamp": now,
            "values": payload_values,
            "events": events,
        }

        try:
            self._post_json("/api/upload/status", payload)
        except (urllib.error.URLError, TimeoutError) as exc:
            self._restore_after_status_failure(payload_values, events)
            self._warn_rate_limited(f"status upload failed: {exc}")
        except Exception as exc:
            self._restore_after_status_failure(payload_values, events)
            self._warn_rate_limited(f"status upload failed: {exc}")

    def _restore_after_status_failure(self, payload_values: Dict[str, Any], events: List[Any]) -> None:
        generated = {"uploader_alive", "uploader_seq", "uploader_time"}
        with self.pending_lock:
            for key, value in payload_values.items():
                if key not in generated:
                    self.pending[key] = value
            for item in events:
                self.event_queue.append(item)

    def _warn_rate_limited(self, text: str) -> None:
        now = time.monotonic()
        if now - self.last_status_error_log > 2.0:
            self.last_status_error_log = now
            self.get_logger().warn(text)

    def _image_callback(self, stream_key: str, msg: Image, source_topic: Optional[str] = None) -> None:
        if not self.video_enabled:
            return

        if source_topic:
            self._set_pending(f"video.{stream_key}_topic", source_topic)
        self.frame_seen_counts[stream_key] = self.frame_seen_counts.get(stream_key, 0) + 1
        self._set_pending(f"video.{stream_key}_seen", self.frame_seen_counts[stream_key])

        now = time.monotonic()
        if now - self.last_video_sent.get(stream_key, 0.0) < self.video_period_s:
            return
        if stream_key in self.video_busy:
            return

        self.last_video_sent[stream_key] = now
        self.video_busy.add(stream_key)

        def worker() -> None:
            try:
                frame = image_to_bgr(msg)
                frame = resize_frame(frame, self.image_width, self.image_height)
                ok, encoded = cv2.imencode(
                    ".jpg",
                    frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)],
                )
                if not ok:
                    raise RuntimeError("jpeg encode failed")
                self._post_jpeg(
                    f"/api/upload/frame/{stream_key}",
                    encoded.tobytes(),
                    timeout_s=max(1.0, self.http_timeout_s),
                )
                self._set_pending(f"video.{stream_key}_last_error", None)
                self._set_pending(f"video.{stream_key}_uploaded", True)
            except Exception as exc:
                error_text = str(exc)
                self._set_pending(f"video.{stream_key}_last_error", error_text)
                self._set_pending(f"video.{stream_key}_uploaded", False)
                self.get_logger().warn(f"frame upload failed for {stream_key}: {error_text}")
            finally:
                self.video_busy.discard(stream_key)

        threading.Thread(target=worker, daemon=True).start()


def parse_args() -> Tuple[argparse.Namespace, List[str]]:
    parser = argparse.ArgumentParser(
        description="Upload-only robot-side uploader for Platooning Raspberry Pi Monitor"
    )
    parser.add_argument("--robot", choices=["leader", "follower"], required=True)
    parser.add_argument("--server", required=True, help="예: http://192.168.0.10:8080")
    parser.add_argument("--token", nargs="?", const="", default="", help="서버 MONITOR_TOKEN 값")
    parser.add_argument("--status-period", type=float, default=0.2)
    parser.add_argument("--video-period", type=float, default=0.25)
    parser.add_argument("--jpeg-quality", type=int, default=65)
    parser.add_argument("--image-width", type=int, default=320)
    parser.add_argument("--image-height", type=int, default=240)
    parser.add_argument("--no-video", action="store_true")
    parser.add_argument("--video-enabled", choices=["true", "false"], default=None)
    parser.add_argument("--http-timeout", type=float, default=1.0)
    return parser.parse_known_args()


def main() -> None:
    args, ros_args = parse_args()
    if args.status_period <= 0:
        raise ValueError("--status-period must be > 0")
    if args.video_period <= 0:
        raise ValueError("--video-period must be > 0")

    if args.video_enabled is None:
        video_enabled = not args.no_video
    else:
        video_enabled = args.video_enabled.lower() == "true"

    rclpy.init(args=ros_args)
    node = RobotStatusUploader(
        robot=args.robot,
        server=args.server,
        token=args.token,
        status_period_s=args.status_period,
        video_period_s=args.video_period,
        jpeg_quality=args.jpeg_quality,
        video_enabled=video_enabled,
        image_width=args.image_width,
        image_height=args.image_height,
        http_timeout_s=args.http_timeout,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
