#!/usr/bin/env python3
"""Leader startup IMU heading alignment with time-based reverse return.

The leader drives straight forward while the host estimates the IMU-to-global yaw
alignment from /leader/global/position. Once /leader/heading_aligned becomes
true, this node reverses for the same duration it drove forward, then publishes a
short stop command and exits.
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class ImuForwardAlign(Node):
    def __init__(self):
        super().__init__('imu_forward_align')

        self.declare_parameter('robot', 'leader')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('heading_aligned_topic', '')
        self.declare_parameter('linear_x', 2.0)
        self.declare_parameter('stop_duration', 0.5)
        self.declare_parameter('max_drive_sec', 15.0)
        self.declare_parameter('publish_rate', 20.0)

        self.robot = str(self.get_parameter('robot').value).strip('/')
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        configured_aligned_topic = str(
            self.get_parameter('heading_aligned_topic').value
        )
        self.heading_aligned_topic = (
            configured_aligned_topic or f'/{self.robot}/heading_aligned'
        )
        self.linear_x = abs(float(self.get_parameter('linear_x').value))
        self.stop_duration = max(
            float(self.get_parameter('stop_duration').value),
            0.0,
        )
        self.max_drive_sec = max(float(self.get_parameter('max_drive_sec').value), 0.0)
        publish_rate = float(self.get_parameter('publish_rate').value)
        publish_rate = publish_rate if publish_rate > 0.0 else 20.0

        self.publisher_ = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        heading_qos = QoSProfile(depth=1)
        heading_qos.reliability = ReliabilityPolicy.RELIABLE
        heading_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.heading_aligned_sub = self.create_subscription(
            Bool,
            self.heading_aligned_topic,
            self.heading_aligned_callback,
            heading_qos,
        )

        self.start_time = time.monotonic()
        self.aligned_time = None
        self.reverse_duration = 0.0
        self.reverse_start_time = None
        self.stop_start_time = None
        self.state = 'FORWARD'
        self.finished = False
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info(
            f'{self.robot} IMU forward alignment started: vx={self.linear_x:.3f}m/s '
            f'until {self.heading_aligned_topic}=true -> reverse same duration '
            f'and stop on {self.cmd_vel_topic} '
            f'(safety stop after {self.max_drive_sec:.1f}s)'
        )

    def heading_aligned_callback(self, msg):
        if not bool(msg.data) or self.aligned_time is not None:
            return
        self.aligned_time = time.monotonic()
        self.reverse_duration = max(self.aligned_time - self.start_time, 0.0)
        self.reverse_start_time = self.aligned_time
        self.state = 'REVERSE'
        self.get_logger().info(
            f'Heading aligned after {self.reverse_duration:.2f}s forward. '
            f'Reversing for {self.reverse_duration:.2f}s.'
        )

    def timer_callback(self):
        now = time.monotonic()

        if self.state == 'FORWARD':
            if (
                self.max_drive_sec > 0.0
                and now - self.start_time > self.max_drive_sec
                and not self.finished
            ):
                self.finished = True
                self._publish_stop()
                self.get_logger().warn(
                    f'Heading not aligned within {self.max_drive_sec:.1f}s; stopping '
                    f'leader without reverse. Check {self.heading_aligned_topic}.'
                )
                rclpy.shutdown()
                return
            self._publish_forward()
            return

        if self.state == 'REVERSE':
            elapsed_reverse = now - self.reverse_start_time
            if elapsed_reverse < self.reverse_duration:
                self._publish_reverse()
                return
            self._publish_stop()
            self.stop_start_time = now
            self.state = 'STOPPING'
            self.get_logger().info('Reverse return finished. Stopping leader.')
            return

        if self.state == 'STOPPING':
            self._publish_stop()
            if now - self.stop_start_time >= self.stop_duration and not self.finished:
                self.finished = True
                self.get_logger().info('IMU forward alignment finished. Leader returned and stopped.')
                rclpy.shutdown()

    def _publish_forward(self):
        twist = Twist()
        twist.linear.x = self.linear_x
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def _publish_reverse(self):
        twist = Twist()
        twist.linear.x = -self.linear_x
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
