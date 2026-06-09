#!/usr/bin/env python3
"""Detect a target region and publish one initial bbox."""

import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

try:
    import cv2
except ImportError:
    cv2 = None


class AutoInitBbox(Node):
    def __init__(self):
        super().__init__("auto_init_bbox")

        self.image_topic = self.declare_parameter("image_topic", "/camera/color/image_raw").value
        self.alternate_image_topics = str(
            self.declare_parameter("alternate_image_topics", "").value)
        self.bbox_topic = self.declare_parameter("bbox_topic", "/target/init_bbox").value
        self.tracked_bbox_topic = str(
            self.declare_parameter("tracked_bbox_topic", "").value)
        self.publish_tracked_bbox = bool(
            self.declare_parameter("publish_tracked_bbox", False).value)
        self.status_topic = self.declare_parameter("status_topic", "/target/auto_init_bbox_status").value
        self.color_mode = self.declare_parameter("color_mode", "black").value
        self.enable_topic = str(self.declare_parameter("enable_topic", "").value)
        self.enabled = bool(self.declare_parameter("start_enabled", True).value)

        self.min_mask_pixels = int(self.declare_parameter("min_mask_pixels", 700).value)
        self.min_bbox_width_px = float(self.declare_parameter("min_bbox_width_px", 20.0).value)
        self.min_bbox_height_px = float(self.declare_parameter("min_bbox_height_px", 20.0).value)
        self.max_bbox_area_ratio = float(self.declare_parameter("max_bbox_area_ratio", 0.65).value)
        self.min_bbox_aspect_ratio = float(
            self.declare_parameter("min_bbox_aspect_ratio", 0.0).value)
        self.max_bbox_aspect_ratio = float(
            self.declare_parameter("max_bbox_aspect_ratio", 1000.0).value)
        self.roi_min_x_ratio = float(self.declare_parameter("roi_min_x_ratio", 0.0).value)
        self.roi_max_x_ratio = float(self.declare_parameter("roi_max_x_ratio", 1.0).value)
        self.roi_min_y_ratio = float(self.declare_parameter("roi_min_y_ratio", 0.0).value)
        self.roi_max_y_ratio = float(self.declare_parameter("roi_max_y_ratio", 1.0).value)
        self.require_bbox_inside_roi = bool(
            self.declare_parameter("require_bbox_inside_roi", False).value)
        self.prefer_center = bool(self.declare_parameter("prefer_center", True).value)
        self.auto_color_min = int(self.declare_parameter("auto_color_min", 55).value)
        self.auto_color_margin = int(self.declare_parameter("auto_color_margin", 35).value)
        self.black_max = int(self.declare_parameter("black_max", 85).value)
        self.black_min_contrast = int(self.declare_parameter("black_min_contrast", 20).value)
        self.depth_unit_scale = float(self.declare_parameter("depth_unit_scale", 0.001).value)
        self.depth_min_m = float(self.declare_parameter("depth_min_m", 0.12).value)
        self.depth_max_m = float(self.declare_parameter("depth_max_m", 1.2).value)
        self.depth_near_percentile = float(
            self.declare_parameter("depth_near_percentile", 8.0).value)
        self.depth_band_m = float(self.declare_parameter("depth_band_m", 0.08).value)
        self.box_depth_band_m = float(self.declare_parameter("box_depth_band_m", 0.08).value)
        self.box_min_fill_ratio = float(self.declare_parameter("box_min_fill_ratio", 0.45).value)
        self.box_max_depth_std_m = float(self.declare_parameter("box_max_depth_std_m", 0.08).value)
        self.box_max_center_distance_ratio = float(
            self.declare_parameter("box_max_center_distance_ratio", 0.24).value)
        self.box_center_weight = float(self.declare_parameter("box_center_weight", 0.55).value)
        self.box_area_weight = float(self.declare_parameter("box_area_weight", 0.30).value)
        self.box_depth_weight = float(self.declare_parameter("box_depth_weight", 0.15).value)
        self.depth_first_front_bbox = bool(
            self.declare_parameter("depth_first_front_bbox", False).value)
        self.require_depth_for_front_yolo = bool(
            self.declare_parameter("require_depth_for_front_yolo", True).value)
        self.depth_yolo_iou_min = float(
            self.declare_parameter("depth_yolo_iou_min", 0.30).value)
        self.depth_yolo_min_pixels = int(
            self.declare_parameter("depth_yolo_min_pixels", 80).value)
        self.depth_yolo_min_fill_ratio = float(
            self.declare_parameter("depth_yolo_min_fill_ratio", 0.02).value)
        self.yolo_as_assist_only = bool(
            self.declare_parameter("yolo_as_assist_only", True).value)
        self.yolo_image_topic = str(self.declare_parameter("yolo_image_topic", "").value)
        self.yolo_image_max_age_s = float(
            self.declare_parameter("yolo_image_max_age_s", 0.75).value)
        self.yolo_model_path = str(self.declare_parameter("yolo_model_path", "").value)
        self.yolo_confidence = float(self.declare_parameter("yolo_confidence", 0.35).value)
        self.yolo_imgsz = int(self.declare_parameter("yolo_imgsz", 640).value)
        self.yolo_class_name = str(self.declare_parameter("yolo_class_name", "box").value)
        self.yolo_max_detections = int(self.declare_parameter("yolo_max_detections", 5).value)
        self.yolo_lock_target = bool(
            self.declare_parameter("yolo_lock_target", True).value)
        self.yolo_min_accept_confidence = float(
            self.declare_parameter("yolo_min_accept_confidence", 0.30).value)
        self.yolo_locked_min_accept_confidence = float(
            self.declare_parameter("yolo_locked_min_accept_confidence", 0.35).value)
        self.yolo_max_center_jump_ratio = float(
            self.declare_parameter("yolo_max_center_jump_ratio", 0.08).value)
        self.yolo_anchor_max_center_jump_ratio = float(
            self.declare_parameter("yolo_anchor_max_center_jump_ratio", 0.18).value)
        self.yolo_min_reselect_iou = float(
            self.declare_parameter("yolo_min_reselect_iou", 0.10).value)
        self.yolo_min_area_ratio_change = float(
            self.declare_parameter("yolo_min_area_ratio_change", 0.50).value)
        self.yolo_max_area_ratio_change = float(
            self.declare_parameter("yolo_max_area_ratio_change", 1.80).value)
        self.yolo_lock_reset_after_misses = int(
            self.declare_parameter("yolo_lock_reset_after_misses", 0).value)
        self.yolo_center_score_weight = float(
            self.declare_parameter("yolo_center_score_weight", 0.35).value)
        self.yolo_area_score_weight = float(
            self.declare_parameter("yolo_area_score_weight", 0.25).value)
        self.bbox_padding_scale_x = max(
            1.0, float(self.declare_parameter("bbox_padding_scale_x", 1.0).value))
        self.bbox_padding_scale_y = max(
            1.0, float(self.declare_parameter("bbox_padding_scale_y", 1.0).value))

        self.publish_repeat = int(self.declare_parameter("publish_repeat", 5).value)
        self.repeat_period_s = float(self.declare_parameter("repeat_period_s", 0.12).value)
        self.continuous_publish = bool(
            self.declare_parameter("continuous_publish", False).value)
        self.continuous_publish_period_s = float(
            self.declare_parameter("continuous_publish_period_s", 0.5).value)
        self.reuse_last_bbox_on_loss = bool(
            self.declare_parameter("reuse_last_bbox_on_loss", False).value)
        self.reuse_last_bbox_max_age_s = float(
            self.declare_parameter("reuse_last_bbox_max_age_s", 0.0).value)
        self.lock_first_bbox = bool(
            self.declare_parameter("lock_first_bbox", False).value)
        self.timeout_s = float(self.declare_parameter("timeout_s", 0.0).value)

        self.red_min = int(self.declare_parameter("red_min", 80).value)
        self.red_margin = int(self.declare_parameter("red_margin", 35).value)
        self.red_ratio = float(self.declare_parameter("red_ratio", 1.25).value)

        self.green_min = int(self.declare_parameter("green_min", 70).value)
        self.green_margin = int(self.declare_parameter("green_margin", 25).value)
        self.green_ratio = float(self.declare_parameter("green_ratio", 1.15).value)

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.bbox_pub = self.create_publisher(Float32MultiArray, self.bbox_topic, latched_qos)
        self.tracked_bbox_pub = None
        if self.publish_tracked_bbox and self.tracked_bbox_topic:
            self.tracked_bbox_pub = self.create_publisher(
                Float32MultiArray, self.tracked_bbox_topic, latched_qos)
        self.status_pub = self.create_publisher(String, self.status_topic, latched_qos)
        self.image_subs = []
        for topic in self.image_topics():
            self.image_subs.append(self.create_subscription(
                Image,
                topic,
                lambda msg, source_topic=topic: self.on_image(msg, source_topic),
                qos_profile_sensor_data))
        self.yolo_image_sub = None
        if self.yolo_image_topic and self.yolo_image_topic not in self.image_topics():
            self.yolo_image_sub = self.create_subscription(
                Image,
                self.yolo_image_topic,
                self.on_yolo_image,
                qos_profile_sensor_data)
        self.enable_sub = None
        if self.enable_topic:
            self.enable_sub = self.create_subscription(
                Bool,
                self.enable_topic,
                self.on_enable,
                latched_qos)
        self.timeout_timer = self.create_timer(0.5, self.on_timeout)
        self.heartbeat_timer = self.create_timer(2.0, self.on_heartbeat)

        self.start_time = time.monotonic()
        self.published = False
        self.last_publish_time = 0.0
        self.last_warn_time = 0.0
        self.logged_first_image = False
        self.last_status = ""
        self.last_bbox = None
        self.last_bbox_update_time = 0.0
        self.anchor_bbox = None
        self.yolo_lock_miss_count = 0
        self.yolo_model = None
        self.yolo_load_failed = False
        self.latest_yolo_image = None
        self.latest_yolo_image_time = 0.0

        self.publish_status(
            f"waiting for {self.color_mode} target on {self.image_topics()}; publishing bbox to {self.bbox_topic}")
        if self.color_mode == "yolo" or self.depth_first_active():
            self.load_yolo_model()

    def on_enable(self, msg):
        was_enabled = self.enabled
        self.enabled = bool(msg.data)
        if self.enabled and not was_enabled:
            self.published = False
            self.last_bbox = None
            self.last_bbox_update_time = 0.0
            self.anchor_bbox = None
            self.yolo_lock_miss_count = 0
            self.latest_yolo_image = None
            self.latest_yolo_image_time = 0.0
            self.last_publish_time = 0.0
            self.start_time = time.monotonic()
            self.publish_status(
                f"enabled target detection on {self.image_topics()}; publishing bbox to {self.bbox_topic}")
        elif not self.enabled and was_enabled:
            self.publish_status(f"disabled target detection for {self.bbox_topic}")

    def image_topics(self):
        topics = [str(self.image_topic)]
        alternates = [
            item.strip()
            for item in self.alternate_image_topics.split(",")
            if item.strip()
        ]
        topics.extend(alternates)
        unique_topics = []
        for topic in topics:
            if topic and topic not in unique_topics:
                unique_topics.append(topic)
        return unique_topics

    def depth_first_active(self):
        return self.color_mode != "yolo" and (
            self.depth_first_front_bbox or self.color_mode == "depth_first_front_bbox")

    def on_yolo_image(self, msg):
        self.latest_yolo_image = msg
        self.latest_yolo_image_time = time.monotonic()

    def publish_status(self, text):
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def on_heartbeat(self):
        if self.published and not self.continuous_publish:
            return
        if self.last_status:
            msg = String()
            msg.data = self.last_status
            self.status_pub.publish(msg)

    def on_timeout(self):
        if (self.published and not self.continuous_publish) or self.timeout_s <= 0.0:
            return
        if time.monotonic() - self.start_time > self.timeout_s:
            self.publish_status("timeout waiting for auto init bbox target")
            self.published = True

    def on_image(self, msg, source_topic=None):
        if not self.enabled:
            return

        if self.published and not self.continuous_publish:
            return

        now = time.monotonic()
        if (
            self.published
            and self.continuous_publish
            and now - self.last_publish_time < self.continuous_publish_period_s
        ):
            return

        if self.lock_first_bbox and self.last_bbox is not None and self.color_mode != "yolo":
            if now - self.last_warn_time >= 1.0:
                self.last_warn_time = now
                self.publish_status(f"locked target bbox: {self.last_bbox}")
            self.publish_bbox_repeated(self.last_bbox)
            self.last_publish_time = time.monotonic()
            self.published = True
            return

        if not self.logged_first_image:
            self.logged_first_image = True
            self.publish_status(
                f"receiving image: topic={source_topic or self.image_topic} encoding={msg.encoding} size={msg.width}x{msg.height}")

        bbox = None
        if self.depth_first_active():
            bbox = self.depth_first_front_bbox_candidate(msg)
            image_width = msg.width
            image_height = msg.height
            if bbox is None:
                self.republish_last_bbox("fresh depth-first front bbox unavailable")
                return
            mask = None
        elif self.color_mode == "box":
            bbox = self.depth_box_bbox(msg)
            image_width = msg.width
            image_height = msg.height
            if bbox is None:
                self.republish_last_bbox("fresh depth box unavailable")
                return
            mask = None
        elif self.color_mode == "yolo":
            bbox = self.yolo_bbox(msg)
            image_width = msg.width
            image_height = msg.height
            if bbox is None:
                self.republish_last_bbox("fresh YOLO box unavailable")
                return
            mask = None
        elif self.color_mode == "depth_near":
            mask = self.depth_near_mask(msg)
            image_width = msg.width
            image_height = msg.height
            if mask is None:
                return
        else:
            image = self.image_to_rgb(msg)
            if image is None:
                return
            mask = self.make_mask(image)
            image_width = image.shape[1]
            image_height = image.shape[0]
            mask = self.apply_roi(mask, image_width, image_height)

        if bbox is None:
            bbox = self.mask_to_bbox(mask, image_width, image_height)
        if bbox is None:
            pixels = int(np.count_nonzero(mask)) if mask is not None else 0
            if self.republish_last_bbox(f"target pixels too small: {pixels}"):
                return
            self.throttled_waiting_status(f"target pixels too small: {pixels}")
            return

        x_min, y_min, width, height, pixels = self.expand_bbox(bbox, image_width, image_height)
        area_ratio = (width * height) / float(max(1, image_width * image_height))

        if width < self.min_bbox_width_px or height < self.min_bbox_height_px:
            self.throttled_waiting_status(f"bbox too small: {width:.1f}x{height:.1f}")
            return

        if area_ratio > self.max_bbox_area_ratio:
            self.throttled_waiting_status(f"bbox too large: area_ratio={area_ratio:.2f}")
            return

        aspect_ratio = width / max(1.0, height)
        if aspect_ratio < self.min_bbox_aspect_ratio or aspect_ratio > self.max_bbox_aspect_ratio:
            self.throttled_waiting_status(
                f"bbox aspect rejected: ratio={aspect_ratio:.2f}")
            return

        bbox_msg = [float(x_min), float(y_min), width, height]
        self.last_bbox = bbox_msg
        self.last_bbox_update_time = time.monotonic()
        self.publish_status(f"auto init bbox detected: {bbox_msg}; pixels={pixels}")
        self.publish_bbox_repeated(bbox_msg)
        self.last_publish_time = time.monotonic()
        self.published = True

    def expand_bbox(self, bbox, image_width, image_height):
        x_min, y_min, width, height, pixels = bbox
        scale_x = self.bbox_padding_scale_x
        scale_y = self.bbox_padding_scale_y
        if scale_x <= 1.0 and scale_y <= 1.0:
            return x_min, y_min, width, height, pixels

        image_width = float(max(1, image_width))
        image_height = float(max(1, image_height))
        center_x = float(x_min) + 0.5 * float(width)
        center_y = float(y_min) + 0.5 * float(height)
        padded_width = min(image_width, max(1.0, float(width) * scale_x))
        padded_height = min(image_height, max(1.0, float(height) * scale_y))

        x0 = center_x - 0.5 * padded_width
        x1 = center_x + 0.5 * padded_width
        y0 = center_y - 0.5 * padded_height
        y1 = center_y + 0.5 * padded_height

        if x0 < 0.0:
            x1 -= x0
            x0 = 0.0
        if x1 > image_width:
            x0 -= x1 - image_width
            x1 = image_width
        if y0 < 0.0:
            y1 -= y0
            y0 = 0.0
        if y1 > image_height:
            y0 -= y1 - image_height
            y1 = image_height

        x0 = float(np.clip(x0, 0.0, image_width - 1.0))
        y0 = float(np.clip(y0, 0.0, image_height - 1.0))
        x1 = float(np.clip(x1, x0 + 1.0, image_width))
        y1 = float(np.clip(y1, y0 + 1.0, image_height))
        return x0, y0, x1 - x0, y1 - y0, pixels

    def throttled_waiting_status(self, reason):
        now = time.monotonic()
        if now - self.last_warn_time < 1.0:
            return
        self.last_warn_time = now
        self.publish_status(f"waiting for target: {reason}")

    def publish_bbox_repeated(self, bbox):
        msg = Float32MultiArray()
        msg.data = bbox
        for _ in range(max(1, self.publish_repeat)):
            self.bbox_pub.publish(msg)
            if self.tracked_bbox_pub is not None:
                self.tracked_bbox_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(max(0.0, self.repeat_period_s))

    def republish_last_bbox(self, reason):
        if not self.reuse_last_bbox_on_loss or self.last_bbox is None:
            return False

        now = time.monotonic()
        if (
            self.reuse_last_bbox_max_age_s > 0.0
            and self.last_bbox_update_time > 0.0
            and now - self.last_bbox_update_time > self.reuse_last_bbox_max_age_s
        ):
            self.throttled_waiting_status(
                f"last bbox too old for reuse: age={now - self.last_bbox_update_time:.2f}s; {reason}")
            return False

        if now - self.last_warn_time >= 1.0:
            self.last_warn_time = now
            self.publish_status(
                f"reusing last bbox: {self.last_bbox}; {reason}")
        self.publish_bbox_repeated(self.last_bbox)
        self.last_publish_time = time.monotonic()
        self.published = True
        return True

    def image_to_rgb(self, msg):
        encoding = msg.encoding.lower()
        if encoding in ("yuv422_yuy2", "yuyv", "yuyv422"):
            return self.yuyv_to_rgb(msg)

        channels = self.encoding_channels(msg.encoding)
        if channels is None:
            self.throttled_waiting_status(f"unsupported image encoding: {msg.encoding}")
            return None

        expected_step = msg.width * channels
        if msg.step < expected_step:
            self.throttled_waiting_status(
                f"invalid image step: step={msg.step}, expected>={expected_step}")
            return None

        data = np.frombuffer(msg.data, dtype=np.uint8)
        row_stride = int(msg.step)
        rows = []
        for row in range(msg.height):
            start = row * row_stride
            end = start + expected_step
            rows.append(data[start:end].reshape((msg.width, channels)))
        image = np.stack(rows, axis=0)

        if encoding in ("bgr8", "bgra8"):
            rgb = image[:, :, [2, 1, 0]]
        elif encoding in ("rgb8", "rgba8"):
            rgb = image[:, :, [0, 1, 2]]
        else:
            return None
        return rgb.astype(np.uint16)

    def yuyv_to_rgb(self, msg):
        expected_step = msg.width * 2
        if msg.step < expected_step:
            self.throttled_waiting_status(
                f"invalid YUYV image step: step={msg.step}, expected>={expected_step}")
            return None

        data = np.frombuffer(msg.data, dtype=np.uint8)
        row_stride = int(msg.step)
        rows = []
        for row in range(msg.height):
            start = row * row_stride
            end = start + expected_step
            rows.append(data[start:end].reshape((msg.width // 2, 4)))
        yuyv = np.stack(rows, axis=0).astype(np.float32)

        y0 = yuyv[:, :, 0]
        u = yuyv[:, :, 1] - 128.0
        y1 = yuyv[:, :, 2]
        v = yuyv[:, :, 3] - 128.0

        rgb0 = self.yuv_to_rgb_pixels(y0, u, v)
        rgb1 = self.yuv_to_rgb_pixels(y1, u, v)
        rgb = np.empty((msg.height, msg.width, 3), dtype=np.uint16)
        rgb[:, 0::2, :] = rgb0
        rgb[:, 1::2, :] = rgb1
        return rgb

    @staticmethod
    def yuv_to_rgb_pixels(y, u, v):
        r = y + 1.402 * v
        g = y - 0.344136 * u - 0.714136 * v
        b = y + 1.772 * u
        return np.stack([
            np.clip(r, 0, 255),
            np.clip(g, 0, 255),
            np.clip(b, 0, 255),
        ], axis=2).astype(np.uint16)

    @staticmethod
    def encoding_channels(encoding):
        encoding = encoding.lower()
        if encoding in ("rgb8", "bgr8"):
            return 3
        if encoding in ("rgba8", "bgra8"):
            return 4
        return None

    def make_mask(self, image):
        r = image[:, :, 0]
        g = image[:, :, 1]
        b = image[:, :, 2]
        max_channel = np.maximum(np.maximum(r, g), b)
        min_channel = np.minimum(np.minimum(r, g), b)

        if self.color_mode == "auto":
            return (
                (max_channel >= self.auto_color_min) &
                ((max_channel - min_channel) >= self.auto_color_margin)
            )

        if self.color_mode == "black":
            luma = ((77 * r) + (150 * g) + (29 * b)) // 256
            scene_luma = float(np.median(luma))
            dark_enough = luma <= self.black_max
            contrasted = (scene_luma - luma) >= self.black_min_contrast
            return dark_enough & contrasted

        if self.color_mode == "green":
            return (
                (g >= self.green_min) &
                (g >= r * self.green_ratio) &
                (g >= b * self.green_ratio) &
                ((g - np.maximum(r, b)) >= self.green_margin)
            )

        return (
            (r >= self.red_min) &
            (r >= g * self.red_ratio) &
            (r >= b * self.red_ratio) &
            ((r - np.maximum(g, b)) >= self.red_margin)
        )

    def depth_near_mask(self, msg):
        depth = self.image_to_depth_m(msg)
        if depth is None:
            return None

        valid = (
            np.isfinite(depth) &
            (depth >= self.depth_min_m) &
            (depth <= self.depth_max_m)
        )
        valid = self.apply_roi(valid, msg.width, msg.height)
        valid_depth = depth[valid]
        if valid_depth.size < self.min_mask_pixels:
            self.throttled_waiting_status(
                f"valid depth pixels in ROI too small: {valid_depth.size}")
            return None

        percentile = min(50.0, max(0.1, self.depth_near_percentile))
        near_depth = float(np.percentile(valid_depth, percentile))
        band = max(0.005, self.depth_band_m)
        mask = valid & (depth <= near_depth + band)
        return mask

    def depth_box_bbox(self, msg):
        depth = self.image_to_depth_m(msg)
        if depth is None:
            return None
        if cv2 is None:
            self.throttled_waiting_status("box mode requires OpenCV connected components")
            return None

        valid = (
            np.isfinite(depth) &
            (depth >= self.depth_min_m) &
            (depth <= self.depth_max_m)
        )
        valid = self.apply_roi(valid, msg.width, msg.height)
        valid_count = int(np.count_nonzero(valid))
        if valid_count < self.min_mask_pixels:
            self.throttled_waiting_status(
                f"valid box depth pixels in ROI too small: {valid_count}")
            return None

        valid_depth = depth[valid]
        percentile = min(50.0, max(0.1, self.depth_near_percentile))
        near_depth = float(np.percentile(valid_depth, percentile))
        band = max(0.005, self.box_depth_band_m)
        candidate_mask = valid & (depth <= near_depth + band)
        candidate_count = int(np.count_nonzero(candidate_mask))
        if candidate_count < self.min_mask_pixels:
            self.throttled_waiting_status(
                f"near-depth box pixels too small: {candidate_count} near={near_depth:.3f} band={band:.3f}")
            return None

        labels_count, labels, stats, _ = cv2.connectedComponentsWithStats(
            candidate_mask.astype(np.uint8), 8)
        if labels_count <= 1:
            self.throttled_waiting_status("no box-like depth components")
            return None

        best = None
        best_score = -1.0
        image_area = float(max(1, msg.width * msg.height))
        image_center_x = 0.5 * float(msg.width)
        image_center_y = 0.5 * float(msg.height)
        image_diag = max(1.0, float(np.hypot(msg.width, msg.height)))

        for index in range(1, labels_count):
            pixels = int(stats[index, cv2.CC_STAT_AREA])
            if pixels < self.min_mask_pixels:
                continue

            x = int(stats[index, cv2.CC_STAT_LEFT])
            y = int(stats[index, cv2.CC_STAT_TOP])
            width = float(stats[index, cv2.CC_STAT_WIDTH])
            height = float(stats[index, cv2.CC_STAT_HEIGHT])
            if width < self.min_bbox_width_px or height < self.min_bbox_height_px:
                continue

            bbox_area = width * height
            area_ratio = bbox_area / image_area
            if area_ratio > self.max_bbox_area_ratio:
                continue

            aspect_ratio = width / max(1.0, height)
            if aspect_ratio < self.min_bbox_aspect_ratio or aspect_ratio > self.max_bbox_aspect_ratio:
                continue

            fill_ratio = float(pixels) / max(1.0, bbox_area)
            if fill_ratio < self.box_min_fill_ratio:
                continue

            component_depths = depth[labels == index]
            component_depths = component_depths[np.isfinite(component_depths)]
            if component_depths.size < self.min_mask_pixels:
                continue
            depth_std = float(np.std(component_depths))
            if depth_std > self.box_max_depth_std_m:
                continue

            center_x = x + 0.5 * width
            center_y = y + 0.5 * height
            center_distance = np.hypot(
                center_x - image_center_x,
                center_y - image_center_y) / image_diag
            if center_distance > self.box_max_center_distance_ratio:
                continue
            center_score = 1.0 - min(1.0, center_distance)
            area_score = min(1.0, area_ratio / max(0.001, self.max_bbox_area_ratio))
            depth_score = 1.0 - min(1.0, depth_std / max(0.001, self.box_max_depth_std_m))
            rectangular_score = min(1.0, fill_ratio)
            score = (
                self.box_center_weight * center_score +
                self.box_area_weight * area_score +
                self.box_depth_weight * depth_score
            ) * rectangular_score

            if score > best_score:
                best_score = score
                best = (x, y, width, height, pixels, fill_ratio, depth_std, score)

        if best is None:
            self.throttled_waiting_status("no rectangular box candidate in depth ROI")
            return None

        x, y, width, height, pixels, fill_ratio, depth_std, score = best
        self.publish_status(
            "box candidate: bbox=[{},{},{:.1f},{:.1f}] pixels={} fill={:.2f} depth_std={:.3f} near={:.3f} band={:.3f} score={:.2f}".format(
                x, y, width, height, pixels, fill_ratio, depth_std, near_depth, band, score))
        return x, y, width, height, pixels

    def depth_support_in_bbox(self, msg, bbox):
        depth = self.image_to_depth_m(msg)
        if depth is None:
            return False, "depth image conversion failed"

        x, y, width, height = [float(v) for v in bbox[:4]]
        x0 = int(max(0, np.floor(x)))
        y0 = int(max(0, np.floor(y)))
        x1 = int(min(msg.width, np.ceil(x + max(1.0, width))))
        y1 = int(min(msg.height, np.ceil(y + max(1.0, height))))
        if x1 <= x0 or y1 <= y0:
            return False, f"invalid bbox crop {self.format_bbox(bbox)}"

        crop = depth[y0:y1, x0:x1]
        valid = (
            np.isfinite(crop) &
            (crop >= self.depth_min_m) &
            (crop <= self.depth_max_m)
        )
        valid_depth = crop[valid]
        min_pixels = max(1, int(self.depth_yolo_min_pixels))
        if valid_depth.size < min_pixels:
            return False, f"valid depth support too small: {valid_depth.size}/{min_pixels}"

        percentile = min(50.0, max(0.1, self.depth_near_percentile))
        near_depth = float(np.percentile(valid_depth, percentile))
        band = max(0.005, self.box_depth_band_m)
        supported_depth = valid_depth[valid_depth <= near_depth + band]
        support_count = int(supported_depth.size)
        if support_count < min_pixels:
            return False, f"near-depth support too small: {support_count}/{min_pixels}"

        crop_area = float(max(1, (x1 - x0) * (y1 - y0)))
        fill_ratio = float(support_count) / crop_area
        min_fill = max(0.0, float(self.depth_yolo_min_fill_ratio))
        if fill_ratio < min_fill:
            return False, f"depth support fill {fill_ratio:.3f} < {min_fill:.3f}"

        std_m = float(np.std(supported_depth)) if support_count > 1 else 0.0
        max_std = max(self.box_max_depth_std_m, 0.12)
        if std_m > max_std:
            return False, f"depth support std {std_m:.3f} > {max_std:.3f}"

        return True, (
            f"depth support ok: pixels={support_count} fill={fill_ratio:.3f} "
            f"near={near_depth:.3f} band={band:.3f} std={std_m:.3f}")

    @staticmethod
    def bbox_center_inside(inner, outer, margin_ratio=0.0):
        ix, iy, iw, ih = [float(v) for v in inner[:4]]
        ox, oy, ow, oh = [float(v) for v in outer[:4]]
        cx = ix + 0.5 * max(0.0, iw)
        cy = iy + 0.5 * max(0.0, ih)
        mx = max(0.0, ow) * max(0.0, margin_ratio)
        my = max(0.0, oh) * max(0.0, margin_ratio)
        return (
            ox - mx <= cx <= ox + max(0.0, ow) + mx and
            oy - my <= cy <= oy + max(0.0, oh) + my
        )

    def depth_first_front_bbox_candidate(self, msg):
        depth_bbox = self.depth_box_bbox(msg)

        if not self.yolo_as_assist_only:
            if depth_bbox is None:
                self.throttled_waiting_status(
                    "depth-first front bbox rejected: depth bbox unavailable")
                return None
            self.publish_status(
                "depth-first front bbox accepted without YOLO assist: "
                f"depth_bbox={self.format_bbox(depth_bbox)}")
            return depth_bbox

        yolo_msg = self.latest_yolo_image
        yolo_age = time.monotonic() - self.latest_yolo_image_time
        if yolo_msg is None or yolo_age > max(0.05, self.yolo_image_max_age_s):
            if depth_bbox is None:
                self.throttled_waiting_status(
                    "depth-first front bbox rejected: depth bbox unavailable and YOLO assist image unavailable")
                return None
            self.publish_status(
                "depth-first front bbox accepted: YOLO assist image unavailable "
                f"age={yolo_age:.2f}s depth_bbox={self.format_bbox(depth_bbox)}")
            return depth_bbox

        yolo_bbox = self.yolo_bbox(yolo_msg, update_lock=False, emit_status=False)
        if yolo_bbox is None:
            if depth_bbox is None:
                self.throttled_waiting_status(
                    "depth-first front bbox rejected: depth bbox unavailable and YOLO assist found no valid box")
                return None
            depth_area_ratio = self.bbox_area_ratio(
                depth_bbox,
                (0.0, 0.0, float(msg.width), float(msg.height))
            )

            if depth_area_ratio > 0.25:
                self.throttled_waiting_status(
                    "depth-first front bbox rejected: YOLO missing and depth bbox too large; "
                    f"area_ratio={depth_area_ratio:.2f} depth_bbox={self.format_bbox(depth_bbox)}")
                return None


            self.publish_status(
                "depth-first front bbox accepted: YOLO assist found no valid box; "
                f"depth_bbox={self.format_bbox(depth_bbox)}")
            return depth_bbox

        scaled_yolo = self.scale_bbox(
            yolo_bbox,
            yolo_msg.width,
            yolo_msg.height,
            msg.width,
            msg.height)
        support_ok, support_reason = self.depth_support_in_bbox(msg, scaled_yolo)
        if depth_bbox is None:
            if support_ok:
                self.publish_status(
                    "depth-first front bbox accepted: YOLO bbox has depth support; "
                    f"yolo_bbox_scaled={self.format_bbox(scaled_yolo)} {support_reason}")
                return scaled_yolo
            self.throttled_waiting_status(
                "depth-first front bbox rejected: depth bbox unavailable; "
                f"YOLO-only publish disabled; {support_reason}")
            return None

        iou = self.bbox_iou(depth_bbox, scaled_yolo)
        center_inside = self.bbox_center_inside(depth_bbox, scaled_yolo, margin_ratio=0.20)
        if iou < max(0.0, self.depth_yolo_iou_min) and not center_inside:
            self.throttled_waiting_status(
                "depth-first front bbox rejected: "
                f"YOLO/depth IoU={iou:.2f} < {self.depth_yolo_iou_min:.2f}; "
                f"depth_bbox={self.format_bbox(depth_bbox)} "
                f"yolo_bbox_scaled={self.format_bbox(scaled_yolo)}")
            return None
        if not support_ok:
            self.throttled_waiting_status(
                "depth-first front bbox rejected: YOLO bbox failed depth support; "
                f"{support_reason} depth_bbox={self.format_bbox(depth_bbox)} "
                f"yolo_bbox_scaled={self.format_bbox(scaled_yolo)}")
            return None

        self.publish_status(
            "depth-first front bbox accepted: "
            f"YOLO/depth IoU={iou:.2f} center_inside={center_inside} "
            f"depth_bbox={self.format_bbox(depth_bbox)} "
            f"yolo_bbox_scaled={self.format_bbox(scaled_yolo)} {support_reason}")
        return scaled_yolo

    def yolo_bbox(self, msg, update_lock=True, emit_status=True):
        model = self.load_yolo_model()
        if model is None:
            return None

        rgb_image = self.image_to_rgb(msg)
        if rgb_image is None:
            return None
        rgb_image = np.clip(rgb_image, 0, 255).astype(np.uint8)
        # Ultralytics treats numpy image sources as OpenCV-style BGR.
        image = rgb_image[:, :, ::-1]

        try:
            results = model.predict(
                source=image,
                conf=max(0.01, min(0.99, self.yolo_confidence)),
                imgsz=max(32, self.yolo_imgsz),
                max_det=max(1, self.yolo_max_detections),
                verbose=False,
            )
        except Exception as exc:
            if emit_status:
                self.throttled_waiting_status(f"YOLO inference failed: {exc}")
            return None

        if not results:
            if emit_status:
                self.throttled_waiting_status("YOLO returned no result")
            return None
        result = results[0]
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            if emit_status:
                self.throttled_waiting_status("YOLO found no box")
            return None

        names = getattr(result, "names", {}) or {}
        image_area = float(max(1, msg.width * msg.height))
        image_center_x = 0.5 * float(msg.width)
        image_center_y = 0.5 * float(msg.height)
        image_diag = max(1.0, float(np.hypot(msg.width, msg.height)))
        best = None
        best_score = -1.0
        candidates = []
        wanted_class = self.yolo_class_name.strip().lower()
        rejected = {
            "small": 0,
            "class": 0,
            "conf": 0,
            "area": 0,
            "aspect": 0,
            "roi": 0,
        }
        seen = 0

        for box in boxes:
            seen += 1
            xyxy = box.xyxy[0].detach().cpu().numpy().astype(float)
            x0, y0, x1, y1 = xyxy
            x0 = float(np.clip(x0, 0.0, msg.width - 1.0))
            y0 = float(np.clip(y0, 0.0, msg.height - 1.0))
            x1 = float(np.clip(x1, x0 + 1.0, msg.width))
            y1 = float(np.clip(y1, y0 + 1.0, msg.height))
            width = x1 - x0
            height = y1 - y0
            if width < self.min_bbox_width_px or height < self.min_bbox_height_px:
                rejected["small"] += 1
                continue

            cls_id = int(box.cls[0].detach().cpu().item()) if box.cls is not None else -1
            cls_name = str(names.get(cls_id, cls_id)).lower()
            if wanted_class and cls_name != wanted_class:
                rejected["class"] += 1
                continue

            area_ratio = (width * height) / image_area
            if area_ratio > self.max_bbox_area_ratio:
                rejected["area"] += 1
                continue
            aspect_ratio = width / max(1.0, height)
            if aspect_ratio < self.min_bbox_aspect_ratio or aspect_ratio > self.max_bbox_aspect_ratio:
                rejected["aspect"] += 1
                continue

            center_x = x0 + 0.5 * width
            center_y = y0 + 0.5 * height
            if not self.point_inside_roi(center_x, center_y, msg.width, msg.height):
                rejected["roi"] += 1
                continue
            if (
                self.require_bbox_inside_roi
                and not self.bbox_inside_roi(x0, y0, x1, y1, msg.width, msg.height)
            ):
                rejected["roi"] += 1
                continue

            confidence = float(box.conf[0].detach().cpu().item()) if box.conf is not None else 0.0
            min_accept_confidence = (
                self.yolo_locked_min_accept_confidence
                if self.yolo_lock_target and update_lock and self.last_bbox is not None
                else self.yolo_min_accept_confidence
            )
            if confidence < min_accept_confidence:
                rejected["conf"] += 1
                continue

            center_distance = np.hypot(
                center_x - image_center_x,
                center_y - image_center_y) / image_diag
            center_score = 1.0 - min(1.0, center_distance)
            area_score = min(1.0, area_ratio / max(0.01, self.max_bbox_area_ratio))
            center_weight = max(0.0, self.yolo_center_score_weight)
            area_weight = max(0.0, self.yolo_area_score_weight)
            base_weight = max(0.0, 1.0 - center_weight - area_weight)
            score = confidence * (
                base_weight + center_weight * center_score + area_weight * area_score)
            candidate = (x0, y0, width, height, int(width * height), confidence, cls_name, score)
            candidates.append(candidate)
            if score > best_score:
                best_score = score
                best = candidate

        if not candidates:
            reject_text = " ".join(
                f"{key}={value}" for key, value in rejected.items() if value)
            if not reject_text:
                reject_text = "none"
            if emit_status:
                self.throttled_waiting_status(
                    f"YOLO found {seen} box(es), all rejected: {reject_text}; "
                    f"limits area<={self.max_bbox_area_ratio:.2f} "
                    f"aspect=[{self.min_bbox_aspect_ratio:.2f},{self.max_bbox_aspect_ratio:.2f}] "
                    f"roi=x[{self.roi_min_x_ratio:.2f},{self.roi_max_x_ratio:.2f}] "
                    f"y[{self.roi_min_y_ratio:.2f},{self.roi_max_y_ratio:.2f}]")
            return None

        if self.yolo_lock_target and update_lock and self.last_bbox is not None:
            locked_best = None
            locked_score = -1.0e9
            max_jump = max(0.01, self.yolo_max_center_jump_ratio)
            anchor_max_jump = max(max_jump, self.yolo_anchor_max_center_jump_ratio)
            min_iou = max(0.0, self.yolo_min_reselect_iou)
            min_area_change = max(0.01, self.yolo_min_area_ratio_change)
            max_area_change = max(min_area_change, self.yolo_max_area_ratio_change)
            anchor_bbox = self.anchor_bbox if self.anchor_bbox is not None else self.last_bbox
            for candidate in candidates:
                candidate_bbox = candidate[:4]
                iou = self.bbox_iou(self.last_bbox, candidate_bbox)
                jump = self.bbox_center_jump_ratio(self.last_bbox, candidate_bbox, msg.width, msg.height)
                anchor_jump = self.bbox_center_jump_ratio(anchor_bbox, candidate_bbox, msg.width, msg.height)
                area_change = self.bbox_area_ratio(candidate_bbox, self.last_bbox)
                if anchor_jump > anchor_max_jump:
                    continue
                if area_change < min_area_change or area_change > max_area_change:
                    continue
                if iou < min_iou and jump > max_jump:
                    continue
                score = candidate[7] + (0.70 * iou) - (0.35 * jump)
                if score > locked_score:
                    locked_score = score
                    locked_best = candidate

            if locked_best is None:
                self.yolo_lock_miss_count += 1
                if (
                    self.yolo_lock_reset_after_misses > 0
                    and self.yolo_lock_miss_count >= self.yolo_lock_reset_after_misses
                ):
                    self.last_bbox = None
                    self.anchor_bbox = None
                    self.yolo_lock_miss_count = 0
                    if emit_status:
                        self.publish_status(
                            "YOLO target lock reset after consecutive misses; accepting best current box")
                else:
                    if emit_status:
                        self.throttled_waiting_status(
                            "YOLO target locked; no same-object box near last bbox")
                    return None
            else:
                self.yolo_lock_miss_count = 0
                best = locked_best
        else:
            self.yolo_lock_miss_count = 0

        x, y, width, height, pixels, confidence, cls_name, _ = best
        if emit_status:
            self.publish_status(
                "YOLO box: class={} conf={:.2f} bbox=[{:.0f},{:.0f},{:.1f},{:.1f}]".format(
                    cls_name, confidence, x, y, width, height))
        if update_lock and self.anchor_bbox is None:
            self.anchor_bbox = [float(x), float(y), float(width), float(height)]
        return int(round(x)), int(round(y)), float(width), float(height), pixels

    @staticmethod
    def format_bbox(bbox):
        return "[{:.1f},{:.1f},{:.1f},{:.1f}]".format(
            float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3]))

    @staticmethod
    def scale_bbox(bbox, from_width, from_height, to_width, to_height):
        sx = float(to_width) / max(1.0, float(from_width))
        sy = float(to_height) / max(1.0, float(from_height))
        return (
            float(bbox[0]) * sx,
            float(bbox[1]) * sy,
            float(bbox[2]) * sx,
            float(bbox[3]) * sy,
            int(max(1.0, float(bbox[2]) * sx) * max(1.0, float(bbox[3]) * sy)),
        )

    @staticmethod
    def bbox_area_ratio(a, b):
        aw = max(0.0, float(a[2]))
        ah = max(0.0, float(a[3]))
        bw = max(0.0, float(b[2]))
        bh = max(0.0, float(b[3]))
        reference_area = max(1.0, bw * bh)
        return (aw * ah) / reference_area

    @staticmethod
    def bbox_iou(a, b):
        ax0, ay0, aw, ah = [float(v) for v in a[:4]]
        bx0, by0, bw, bh = [float(v) for v in b[:4]]
        ax1 = ax0 + max(0.0, aw)
        ay1 = ay0 + max(0.0, ah)
        bx1 = bx0 + max(0.0, bw)
        by1 = by0 + max(0.0, bh)
        ix0 = max(ax0, bx0)
        iy0 = max(ay0, by0)
        ix1 = min(ax1, bx1)
        iy1 = min(ay1, by1)
        inter = max(0.0, ix1 - ix0) * max(0.0, iy1 - iy0)
        union = max(0.0, aw) * max(0.0, ah) + max(0.0, bw) * max(0.0, bh) - inter
        if union <= 0.0:
            return 0.0
        return inter / union

    @staticmethod
    def bbox_center_jump_ratio(a, b, image_width, image_height):
        ax, ay, aw, ah = [float(v) for v in a[:4]]
        bx, by, bw, bh = [float(v) for v in b[:4]]
        acx = ax + 0.5 * aw
        acy = ay + 0.5 * ah
        bcx = bx + 0.5 * bw
        bcy = by + 0.5 * bh
        distance = float(np.hypot(acx - bcx, acy - bcy))
        image_diag = max(1.0, float(np.hypot(image_width, image_height)))
        return distance / image_diag

    def load_yolo_model(self):
        if self.yolo_model is not None:
            return self.yolo_model
        if self.yolo_load_failed:
            return None
        if not self.yolo_model_path:
            self.yolo_load_failed = True
            self.publish_status("YOLO model path is empty")
            return None

        try:
            from ultralytics import YOLO
            self.yolo_model = YOLO(self.yolo_model_path)
            self.publish_status(f"YOLO model loaded: {self.yolo_model_path}")
        except Exception as exc:
            self.yolo_load_failed = True
            self.publish_status(f"failed to load YOLO model: {exc}")
            return None
        return self.yolo_model

    def apply_roi(self, mask, image_width, image_height):
        x0 = int(np.floor(np.clip(self.roi_min_x_ratio, 0.0, 1.0) * image_width))
        x1 = int(np.ceil(np.clip(self.roi_max_x_ratio, 0.0, 1.0) * image_width))
        y0 = int(np.floor(np.clip(self.roi_min_y_ratio, 0.0, 1.0) * image_height))
        y1 = int(np.ceil(np.clip(self.roi_max_y_ratio, 0.0, 1.0) * image_height))

        x0 = max(0, min(x0, image_width))
        x1 = max(0, min(x1, image_width))
        y0 = max(0, min(y0, image_height))
        y1 = max(0, min(y1, image_height))

        if x1 <= x0 or y1 <= y0:
            return mask

        roi_mask = np.zeros_like(mask, dtype=bool)
        roi_mask[y0:y1, x0:x1] = mask[y0:y1, x0:x1]
        return roi_mask

    def point_inside_roi(self, x, y, image_width, image_height):
        x0 = float(np.clip(self.roi_min_x_ratio, 0.0, 1.0) * image_width)
        x1 = float(np.clip(self.roi_max_x_ratio, 0.0, 1.0) * image_width)
        y0 = float(np.clip(self.roi_min_y_ratio, 0.0, 1.0) * image_height)
        y1 = float(np.clip(self.roi_max_y_ratio, 0.0, 1.0) * image_height)
        return x0 <= x <= x1 and y0 <= y <= y1

    def bbox_inside_roi(self, x0, y0, x1, y1, image_width, image_height):
        roi_x0 = float(np.clip(self.roi_min_x_ratio, 0.0, 1.0) * image_width)
        roi_x1 = float(np.clip(self.roi_max_x_ratio, 0.0, 1.0) * image_width)
        roi_y0 = float(np.clip(self.roi_min_y_ratio, 0.0, 1.0) * image_height)
        roi_y1 = float(np.clip(self.roi_max_y_ratio, 0.0, 1.0) * image_height)
        return roi_x0 <= x0 <= x1 <= roi_x1 and roi_y0 <= y0 <= y1 <= roi_y1

    def image_to_depth_m(self, msg):
        encoding = msg.encoding.lower()
        if encoding in ("16uc1", "mono16"):
            depth = self.strided_image_array(msg, np.uint16, 2)
            if depth is None:
                return None
            if msg.is_bigendian != (sys.byteorder == "big"):
                depth = depth.byteswap()
            return depth.astype(np.float32) * self.depth_unit_scale

        if encoding == "32fc1":
            depth = self.strided_image_array(msg, np.float32, 4)
            if depth is None:
                return None
            if msg.is_bigendian != (sys.byteorder == "big"):
                depth = depth.byteswap()
            return depth.astype(np.float32)

        self.throttled_waiting_status(f"unsupported depth encoding: {msg.encoding}")
        return None

    def strided_image_array(self, msg, dtype, bytes_per_pixel):
        expected_step = msg.width * bytes_per_pixel
        if msg.step < expected_step:
            self.throttled_waiting_status(
                f"invalid image step: step={msg.step}, expected>={expected_step}")
            return None
        try:
            return np.ndarray(
                shape=(msg.height, msg.width),
                dtype=dtype,
                buffer=msg.data,
                strides=(msg.step, bytes_per_pixel),
            ).copy()
        except (TypeError, ValueError) as exc:
            self.throttled_waiting_status(f"invalid image buffer: {exc}")
            return None

    def mask_to_bbox(self, mask, image_width, image_height):
        pixels = int(np.count_nonzero(mask))
        if pixels < self.min_mask_pixels:
            return None

        if cv2 is not None:
            labels_count, labels, stats, _ = cv2.connectedComponentsWithStats(
                mask.astype(np.uint8), 8)
            if labels_count <= 1:
                return None
            component_index = self.choose_component(stats, image_width, image_height)
            component_pixels = int(stats[component_index, cv2.CC_STAT_AREA])
            if component_pixels < self.min_mask_pixels:
                return None
            x = int(stats[component_index, cv2.CC_STAT_LEFT])
            y = int(stats[component_index, cv2.CC_STAT_TOP])
            width = float(stats[component_index, cv2.CC_STAT_WIDTH])
            height = float(stats[component_index, cv2.CC_STAT_HEIGHT])
            return x, y, width, height, component_pixels

        ys, xs = np.nonzero(mask)
        if xs.size < self.min_mask_pixels:
            return None
        x_min = int(xs.min())
        x_max = int(xs.max())
        y_min = int(ys.min())
        y_max = int(ys.max())
        return x_min, y_min, float(x_max - x_min + 1), float(y_max - y_min + 1), pixels

    def choose_component(self, stats, image_width, image_height):
        if not self.prefer_center:
            return 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))

        image_center_x = 0.5 * float(image_width)
        image_center_y = 0.5 * float(image_height)
        image_diag = max(1.0, float(np.hypot(image_width, image_height)))
        best_index = 1
        best_score = -1.0

        for index in range(1, stats.shape[0]):
            area = float(stats[index, cv2.CC_STAT_AREA])
            if area < self.min_mask_pixels:
                continue
            x = float(stats[index, cv2.CC_STAT_LEFT])
            y = float(stats[index, cv2.CC_STAT_TOP])
            width = float(stats[index, cv2.CC_STAT_WIDTH])
            height = float(stats[index, cv2.CC_STAT_HEIGHT])
            center_x = x + 0.5 * width
            center_y = y + 0.5 * height
            center_distance = np.hypot(center_x - image_center_x, center_y - image_center_y) / image_diag
            score = area * (1.0 - min(0.85, center_distance))
            if score > best_score:
                best_score = score
                best_index = index

        return best_index


def main(args=None):
    rclpy.init(args=args)
    node = AutoInitBbox()
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
