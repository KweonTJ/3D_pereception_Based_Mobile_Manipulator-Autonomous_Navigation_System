#!/usr/bin/env python3
"""Leader startup IMU heading alignment (course-over-ground bootstrap).

The leader IMU yaw carries an unknown offset (and drifts while driving), so the
heading is bootstrapped from course-over-ground: on startup the leader drives
straight forward at a slow, steady speed while the HOST estimates the IMU->global
yaw offset from /leader/global/position. The host publishes
/leader/heading_aligned=true once that offset is locked; this node then stops the
leader. The COG computation lives on the host -- this node only drives forward and
stops on the aligned signal.

Ported from the follower's imu_forward_align with leader-specific changes:
  - heading_aligned topic defaults to /leader/heading_aligned
  - lower forward speed for the heavier 4WD + OpenManipulator platform
  - a max_drive_sec safety stop so a missing/late aligned signal cannot drive the
    leader into a wall indefinitely (the follower original had no such bound).
"""
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class ImuForwardAlign(Node):
    def __init__(self):
        super().__init__('imu_forward_align')

        self.declare_parameter('robot', 'leader')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('heading_aligned_topic', '')
        self.declare_parameter('linear_x', 0.08)
        self.declare_parameter('stop_duration', 0.5)
        self.declare_parameter('max_drive_sec', 15.0)
        self.declare_parameter('publish_rate', 20.0)

        self.robot = str(self.get_parameter('robot').value)
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        configured_aligned_topic = self.get_parameter('heading_aligned_topic').value
        self.heading_aligned_topic = (
            configured_aligned_topic or f'/{self.robot}/heading_aligned'
        )
        self.linear_x = float(self.get_parameter('linear_x').value)
        self.stop_duration = max(
            float(self.get_parameter('stop_duration').value),
            0.0,
        )
        self.max_drive_sec = max(float(self.get_parameter('max_drive_sec').value), 0.0)
        publish_rate = float(self.get_parameter('publish_rate').value)
        publish_rate = publish_rate if publish_rate > 0.0 else 20.0

        self.publisher_ = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.heading_aligned_sub = self.create_subscription(
            Bool,
            self.heading_aligned_topic,
            self.heading_aligned_callback,
            10,
        )
        self.start_time = time.monotonic()
        self.aligned_time = None
        self.finished = False
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info(
            f'{self.robot} IMU forward alignment started: vx={self.linear_x:.3f}m/s '
            f'until {self.heading_aligned_topic}=true -> {self.cmd_vel_topic} '
            f'(safety stop after {self.max_drive_sec:.1f}s)'
        )

    def heading_aligned_callback(self, msg):
        if bool(msg.data) and self.aligned_time is None:
            self.aligned_time = time.monotonic()
            self.get_logger().info('Heading aligned. Stopping leader.')

    def timer_callback(self):
        if self.aligned_time is None:
            if (
                self.max_drive_sec > 0.0
                and time.monotonic() - self.start_time > self.max_drive_sec
                and not self.finished
            ):
                self.finished = True
                self._publish_stop()
                self.get_logger().warn(
                    f'Heading not aligned within {self.max_drive_sec:.1f}s; stopping '
                    f'leader. Check the {self.heading_aligned_topic} source on the host.'
                )
                rclpy.shutdown()
                return
            twist = Twist()
            twist.linear.x = self.linear_x
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            return

        elapsed_since_aligned = time.monotonic() - self.aligned_time
        self._publish_stop()

        if elapsed_since_aligned >= self.stop_duration and not self.finished:
            self.finished = True
            self.get_logger().info('IMU forward alignment finished. Leader stopped.')
            rclpy.shutdown()

    def _publish_stop(self):
        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        self.publisher_.publish(stop)


def main(args=None):
    rclpy.init(args=args)
    node = ImuForwardAlign()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
