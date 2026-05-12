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
        self.color_mode = self.declare_parameter("color_mode", "black").value

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

        self.publish_repeat = int(self.declare_parameter("publish_repeat", 5).value)
        self.repeat_period_s = float(self.declare_parameter("repeat_period_s", 0.12).value)
        self.continuous_publish = bool(
            self.declare_parameter("continuous_publish", False).value)
        self.continuous_publish_period_s = float(
            self.declare_parameter("continuous_publish_period_s", 0.5).value)
        self.reuse_last_bbox_on_loss = bool(
            self.declare_parameter("reuse_last_bbox_on_loss", False).value)
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
        self.status_pub = self.create_publisher(String, self.status_topic, latched_qos)
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.on_image, qos_profile_sensor_data)
        self.timeout_timer = self.create_timer(0.5, self.on_timeout)
        self.heartbeat_timer = self.create_timer(2.0, self.on_heartbeat)

        self.start_time = time.monotonic()
        self.published = False
        self.last_publish_time = 0.0
        self.last_warn_time = 0.0
        self.logged_first_image = False
        self.last_status = ""
        self.last_bbox = None

        self.publish_status(
            f"waiting for {self.color_mode} target on {self.image_topic}; publishing bbox to {self.bbox_topic}")

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

    def on_image(self, msg):
        if self.published and not self.continuous_publish:
            return

        now = time.monotonic()
        if (
            self.published
            and self.continuous_publish
            and now - self.last_publish_time < self.continuous_publish_period_s
        ):
            return

        bbox = None
        if self.color_mode == "box":
            bbox = self.depth_box_bbox(msg)
            image_width = msg.width
            image_height = msg.height
            if bbox is None:
                self.republish_last_bbox("fresh depth box unavailable")
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

        if not self.logged_first_image:
            self.logged_first_image = True
            self.publish_status(
                f"receiving image: encoding={msg.encoding} size={msg.width}x{msg.height}")

        if bbox is None:
            bbox = self.mask_to_bbox(mask, image_width, image_height)
        if bbox is None:
            pixels = int(np.count_nonzero(mask)) if mask is not None else 0
            if self.republish_last_bbox(f"target pixels too small: {pixels}"):
                return
            self.throttled_waiting_status(f"target pixels too small: {pixels}")
            return

        x_min, y_min, width, height, pixels = bbox
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
        self.publish_status(f"auto init bbox detected: {bbox_msg}; pixels={pixels}")
        self.publish_bbox_repeated(bbox_msg)
        self.last_publish_time = time.monotonic()
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

    def republish_last_bbox(self, reason):
        if not self.reuse_last_bbox_on_loss or self.last_bbox is None:
            return False

        now = time.monotonic()
        if now - self.last_warn_time >= 1.0:
            self.last_warn_time = now
            self.publish_status(
                f"reusing last depth bbox: {self.last_bbox}; {reason}")
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
