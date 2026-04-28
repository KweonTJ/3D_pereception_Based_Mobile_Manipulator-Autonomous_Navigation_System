#!/usr/bin/env python3

import copy
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, RegionOfInterest


class CameraInfoFromJsonNode(Node):
    def __init__(self) -> None:
        super().__init__('camera_info_from_json')

        self.declare_parameter('json_path', '')
        self.declare_parameter('camera_info_key', 'camera_info')

        self._json_path = Path(self.get_parameter('json_path').get_parameter_value().string_value)
        self._camera_info_key = self.get_parameter('camera_info_key').get_parameter_value().string_value
        self._json_mtime_ns = None
        self._template_msg = None
        self._size_warned = False

        self._load_camera_info(force=True)

        self._camera_info_pub = self.create_publisher(
            CameraInfo,
            'camera_info',
            qos_profile_sensor_data,
        )
        self._image_sub = self.create_subscription(
            Image,
            'image',
            self._image_callback,
            qos_profile_sensor_data,
        )
        self._reload_timer = self.create_timer(1.0, self._reload_if_needed)

    def _reload_if_needed(self) -> None:
        self._load_camera_info(force=False)

    def _load_camera_info(self, force: bool) -> None:
        if not self._json_path.is_file():
            message = f'Calibration JSON not found: {self._json_path}'
            if force:
                raise FileNotFoundError(message)
            self.get_logger().warn(message)
            return

        stat = self._json_path.stat()
        if not force and self._json_mtime_ns == stat.st_mtime_ns:
            return

        data = json.loads(self._json_path.read_text(encoding='utf-8'))
        if self._camera_info_key not in data:
            raise KeyError(
                f'Key "{self._camera_info_key}" not found in calibration JSON: {self._json_path}'
            )

        info = data[self._camera_info_key]
        msg = CameraInfo()
        msg.width = int(info['width'])
        msg.height = int(info['height'])
        msg.distortion_model = info['distortion_model']
        msg.d = list(info['d'])
        msg.k = list(info['k'])
        msg.r = list(info['r'])
        msg.p = list(info['p'])
        msg.binning_x = int(info.get('binning_x', 0))
        msg.binning_y = int(info.get('binning_y', 0))

        roi = info.get('roi', {})
        msg.roi = RegionOfInterest(
            x_offset=int(roi.get('x_offset', 0)),
            y_offset=int(roi.get('y_offset', 0)),
            height=int(roi.get('height', 0)),
            width=int(roi.get('width', 0)),
            do_rectify=bool(roi.get('do_rectify', False)),
        )

        self._template_msg = msg
        self._json_mtime_ns = stat.st_mtime_ns
        self._size_warned = False

        self.get_logger().info(
            f'Loaded calibration from {self._json_path} '
            f'using key "{self._camera_info_key}" ({msg.width}x{msg.height})'
        )

    def _image_callback(self, image_msg: Image) -> None:
        if self._template_msg is None:
            return

        if not self._size_warned and (
            image_msg.width != self._template_msg.width or
            image_msg.height != self._template_msg.height
        ):
            self.get_logger().warn(
                'Incoming image size does not match calibration JSON: '
                f'image={image_msg.width}x{image_msg.height}, '
                f'calibration={self._template_msg.width}x{self._template_msg.height}'
            )
            self._size_warned = True

        camera_info_msg = copy.deepcopy(self._template_msg)
        camera_info_msg.header = image_msg.header
        self._camera_info_pub.publish(camera_info_msg)


def main() -> None:
    rclpy.init()
    node = CameraInfoFromJsonNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
