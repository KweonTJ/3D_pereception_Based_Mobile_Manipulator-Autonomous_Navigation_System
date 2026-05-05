#!/usr/bin/env python3
"""Detect a colored target in the camera image and publish one initial bbox."""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
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
        self.bbox_topic = self.declare_parameter("bbox_topic", "/target/init_bbox").value
        self.status_topic = self.declare_parameter("status_topic", "/target/auto_init_bbox_status").value
        self.color_mode = self.declare_parameter("color_mode", "auto").value

        self.min_mask_pixels = int(self.declare_parameter("min_mask_pixels", 700).value)
        self.min_bbox_width_px = float(self.declare_parameter("min_bbox_width_px", 20.0).value)
        self.min_bbox_height_px = float(self.declare_parameter("min_bbox_height_px", 20.0).value)
        self.max_bbox_area_ratio = float(self.declare_parameter("max_bbox_area_ratio", 0.65).value)
        self.auto_color_min = int(self.declare_parameter("auto_color_min", 55).value)
        self.auto_color_margin = int(self.declare_parameter("auto_color_margin", 35).value)

        self.publish_repeat = int(self.declare_parameter("publish_repeat", 5).value)
        self.repeat_period_s = float(self.declare_parameter("repeat_period_s", 0.12).value)
        self.timeout_s = float(self.declare_parameter("timeout_s", 0.0).value)

        self.red_min = int(self.declare_parameter("red_min", 80).value)
        self.red_margin = int(self.declare_parameter("red_margin", 35).value)
        self.red_ratio = float(self.declare_parameter("red_ratio", 1.25).value)

        self.green_min = int(self.declare_parameter("green_min", 70).value)
        self.green_margin = int(self.declare_parameter("green_margin", 25).value)
        self.green_ratio = float(self.declare_parameter("green_ratio", 1.15).value)

        self.bbox_pub = self.create_publisher(Float32MultiArray, self.bbox_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.on_image, qos_profile_sensor_data)
        self.timeout_timer = self.create_timer(0.5, self.on_timeout)

        self.start_time = time.monotonic()
        self.published = False
        self.last_warn_time = 0.0
        self.logged_first_image = False

        self.publish_status(
            f"waiting for {self.color_mode} target on {self.image_topic}; publishing bbox to {self.bbox_topic}")

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def on_timeout(self):
        if self.published or self.timeout_s <= 0.0:
            return
        if time.monotonic() - self.start_time > self.timeout_s:
            self.publish_status("timeout waiting for auto init bbox target")
            self.published = True

    def on_image(self, msg):
        if self.published:
            return

        image = self.image_to_rgb(msg)
        if image is None:
            return
        if not self.logged_first_image:
            self.logged_first_image = True
            self.publish_status(
                f"receiving image: encoding={msg.encoding} size={msg.width}x{msg.height}")

        mask = self.make_mask(image)
        bbox = self.mask_to_bbox(mask)
        if bbox is None:
            pixels = int(np.count_nonzero(mask))
            self.throttled_waiting_status(f"target pixels too small: {pixels}")
            return

        x_min, y_min, width, height, pixels = bbox
        area_ratio = (width * height) / float(max(1, image.shape[0] * image.shape[1]))

        if width < self.min_bbox_width_px or height < self.min_bbox_height_px:
            self.throttled_waiting_status(f"bbox too small: {width:.1f}x{height:.1f}")
            return

        if area_ratio > self.max_bbox_area_ratio:
            self.throttled_waiting_status(f"bbox too large: area_ratio={area_ratio:.2f}")
            return

        bbox_msg = [float(x_min), float(y_min), width, height]
        self.publish_status(f"auto init bbox detected: {bbox_msg}; pixels={pixels}")
        self.publish_bbox_repeated(bbox_msg)
        self.published = True

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
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(max(0.0, self.repeat_period_s))

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

    def mask_to_bbox(self, mask):
        pixels = int(np.count_nonzero(mask))
        if pixels < self.min_mask_pixels:
            return None

        if cv2 is not None:
            labels_count, labels, stats, _ = cv2.connectedComponentsWithStats(
                mask.astype(np.uint8), 8)
            if labels_count <= 1:
                return None
            component_index = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
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
