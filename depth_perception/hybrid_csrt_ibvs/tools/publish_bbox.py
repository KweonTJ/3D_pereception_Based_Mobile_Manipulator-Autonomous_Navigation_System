#!/usr/bin/env python3
"""Publish one CSRT initialization bbox: [x, y, width, height]."""

import argparse
import time

import rclpy
from std_msgs.msg import Float32MultiArray


def main():
    parser = argparse.ArgumentParser(description="Publish a CSRT initialization bounding box.")
    parser.add_argument("x", type=float, help="Top-left x in image pixels")
    parser.add_argument("y", type=float, help="Top-left y in image pixels")
    parser.add_argument("w", type=float, help="Bounding-box width in pixels")
    parser.add_argument("h", type=float, help="Bounding-box height in pixels")
    parser.add_argument("--topic", default="/target/init_bbox", help="BBox topic")
    parser.add_argument("--repeat", type=int, default=3, help="Number of repeated publishes")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("publish_csrt_bbox")
    pub = node.create_publisher(Float32MultiArray, args.topic, 10)

    msg = Float32MultiArray()
    msg.data = [args.x, args.y, args.w, args.h]

    # Give discovery a moment, then publish several times so late subscribers still receive it.
    time.sleep(0.3)
    for _ in range(max(1, args.repeat)):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
