#!/usr/bin/env python3
"""Capture synchronized Front Astra RGB + EEF USB camera image pairs."""

import argparse
import csv
import sys
from pathlib import Path

import cv2
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image


def stamp_to_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Capture synchronized front/eef camera image pairs for stereo calibration."
    )
    parser.add_argument("--front-topic", default="/camera/color/image_raw")
    parser.add_argument("--eef-topic", default="/eef_camera/image_raw")
    parser.add_argument("--output-dir", default="calibration/stereo/images")
    parser.add_argument("--queue-size", type=int, default=20)
    parser.add_argument("--slop", type=float, default=0.08, help="Approximate sync tolerance in seconds")
    parser.add_argument("--manual", action="store_true", help="Show preview and save only when 's' is pressed")
    parser.add_argument("--display", action="store_true", help="Show preview window in auto-save mode")
    parser.add_argument("--save-every-s", type=float, default=0.5, help="Minimum interval between auto-saved pairs")
    parser.add_argument("--max-pairs", type=int, default=0, help="Stop after N saved pairs; 0 means unlimited")
    parser.add_argument("--encoding", default="bgr8", help="cv_bridge output encoding")
    return parser.parse_args(argv)


class StereoPairCapture(Node):
    def __init__(self, args):
        super().__init__("front_eef_stereo_pair_capture")
        self.args = args
        self.bridge = CvBridge()
        self.output_dir = Path(args.output_dir)
        self.front_dir = self.output_dir / "front"
        self.eef_dir = self.output_dir / "eef"
        self.front_dir.mkdir(parents=True, exist_ok=True)
        self.eef_dir.mkdir(parents=True, exist_ok=True)
        self.metadata_path = self.output_dir / "pairs.csv"
        self.metadata_file = self.metadata_path.open("a", newline="", encoding="utf-8")
        self.metadata_writer = csv.DictWriter(
            self.metadata_file,
            fieldnames=[
                "index",
                "front_image",
                "eef_image",
                "front_stamp_ns",
                "eef_stamp_ns",
                "stamp_delta_s",
                "front_width",
                "front_height",
                "eef_width",
                "eef_height",
            ],
        )
        if self.metadata_path.stat().st_size == 0:
            self.metadata_writer.writeheader()

        self.pair_index = self.next_pair_index()
        self.saved_this_run = 0
        self.last_save_time = self.get_clock().now()
        self.latest_front = None
        self.latest_eef = None
        self.latest_front_msg = None
        self.latest_eef_msg = None

        self.front_sub = Subscriber(
            self,
            Image,
            args.front_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.eef_sub = Subscriber(
            self,
            Image,
            args.eef_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.sync = ApproximateTimeSynchronizer(
            [self.front_sub, self.eef_sub],
            queue_size=max(1, args.queue_size),
            slop=max(0.0, args.slop),
        )
        self.sync.registerCallback(self.on_pair)

        if args.manual or args.display:
            self.preview_timer = self.create_timer(0.03, self.show_preview)
        else:
            self.preview_timer = None

        self.get_logger().info(
            f"Capturing stereo pairs: front={args.front_topic}, eef={args.eef_topic}, "
            f"output={self.output_dir}, manual={args.manual}"
        )

    def next_pair_index(self):
        existing = sorted(self.front_dir.glob("front_*.png"))
        if not existing:
            return 0
        last = existing[-1].stem.split("_")[-1]
        try:
            return int(last) + 1
        except ValueError:
            return len(existing)

    def on_pair(self, front_msg, eef_msg):
        try:
            front = self.bridge.imgmsg_to_cv2(front_msg, desired_encoding=self.args.encoding)
            eef = self.bridge.imgmsg_to_cv2(eef_msg, desired_encoding=self.args.encoding)
        except Exception as exc:
            self.get_logger().warning(f"cv_bridge conversion failed: {exc}")
            return

        self.latest_front = front
        self.latest_eef = eef
        self.latest_front_msg = front_msg
        self.latest_eef_msg = eef_msg

        if not self.args.manual:
            elapsed = (self.get_clock().now() - self.last_save_time).nanoseconds * 1e-9
            if elapsed >= max(0.0, self.args.save_every_s):
                self.save_pair(front_msg, eef_msg, front, eef)

    def save_pair(self, front_msg, eef_msg, front, eef):
        index = self.pair_index
        front_path = self.front_dir / f"front_{index:03d}.png"
        eef_path = self.eef_dir / f"eef_{index:03d}.png"
        if not cv2.imwrite(str(front_path), front):
            self.get_logger().error(f"Failed to write {front_path}")
            return
        if not cv2.imwrite(str(eef_path), eef):
            self.get_logger().error(f"Failed to write {eef_path}")
            return

        front_stamp_ns = stamp_to_ns(front_msg.header.stamp)
        eef_stamp_ns = stamp_to_ns(eef_msg.header.stamp)
        self.metadata_writer.writerow(
            {
                "index": index,
                "front_image": str(front_path),
                "eef_image": str(eef_path),
                "front_stamp_ns": front_stamp_ns,
                "eef_stamp_ns": eef_stamp_ns,
                "stamp_delta_s": abs(front_stamp_ns - eef_stamp_ns) * 1e-9,
                "front_width": int(front.shape[1]),
                "front_height": int(front.shape[0]),
                "eef_width": int(eef.shape[1]),
                "eef_height": int(eef.shape[0]),
            }
        )
        self.metadata_file.flush()
        self.get_logger().info(
            f"Saved pair {index:03d}: front={front.shape[1]}x{front.shape[0]}, "
            f"eef={eef.shape[1]}x{eef.shape[0]}, "
            f"dt={abs(front_stamp_ns - eef_stamp_ns) * 1e-9:.4f}s"
        )
        self.pair_index += 1
        self.saved_this_run += 1
        self.last_save_time = self.get_clock().now()

        if self.args.max_pairs > 0 and self.saved_this_run >= self.args.max_pairs:
            self.get_logger().info(f"Reached max pair count: {self.args.max_pairs}")
            rclpy.shutdown()

    def show_preview(self):
        if self.latest_front is None or self.latest_eef is None:
            return
        front = self.latest_front
        eef = self.latest_eef
        target_height = min(front.shape[0], eef.shape[0], 480)
        front_view = cv2.resize(front, (int(front.shape[1] * target_height / front.shape[0]), target_height))
        eef_view = cv2.resize(eef, (int(eef.shape[1] * target_height / eef.shape[0]), target_height))
        view = cv2.hconcat([front_view, eef_view])
        cv2.putText(
            view,
            f"pairs={self.pair_index}  press s=save q=quit",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow("front/eef stereo pair capture", view)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("s") and self.latest_front_msg is not None and self.latest_eef_msg is not None:
            self.save_pair(self.latest_front_msg, self.latest_eef_msg, self.latest_front, self.latest_eef)
        elif key in (ord("q"), 27):
            rclpy.shutdown()

    def destroy_node(self):
        self.metadata_file.close()
        if self.args.manual or self.args.display:
            cv2.destroyAllWindows()
        super().destroy_node()


def main():
    args = parse_args(remove_ros_args(args=sys.argv)[1:])
    rclpy.init(args=[sys.argv[0]])
    node = StereoPairCapture(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
