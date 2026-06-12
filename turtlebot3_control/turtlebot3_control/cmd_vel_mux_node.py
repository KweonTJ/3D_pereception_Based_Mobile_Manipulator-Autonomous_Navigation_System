#!/usr/bin/env python3
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String


class CmdVelMuxNode(Node):
    """Single writer for the real /cmd_vel topic.

    NAV mode forwards /leader/cmd_vel. PICK mode forwards the manipulator-side
    command topic, but /target/base_hold always wins and publishes zero.
    """

    VALID_MODES = {'NAV', 'PICK', 'HOLD', 'STOP'}

    def __init__(self):
        super().__init__('cmd_vel_mux_node')

        self.declare_parameter('nav_cmd_vel_topic', '/leader/cmd_vel')
        self.declare_parameter('pick_cmd_vel_topic', '/turtlebot3_control/pick_cmd_vel')
        self.declare_parameter('output_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('mode_topic', '/turtlebot3_control/mux_mode')
        self.declare_parameter('base_hold_topic', '/target/base_hold')
        self.declare_parameter('status_topic', '/turtlebot3_control/mux_status')
        self.declare_parameter('default_mode', 'NAV')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('nav_cmd_timeout_s', 0.5)
        self.declare_parameter('pick_cmd_timeout_s', 0.5)

        self.nav_cmd_timeout_s = float(self.get_parameter('nav_cmd_timeout_s').value)
        self.pick_cmd_timeout_s = float(self.get_parameter('pick_cmd_timeout_s').value)
        self.mode = self._normalize_mode(str(self.get_parameter('default_mode').value))

        self.latest_nav_cmd = Twist()
        self.latest_pick_cmd = Twist()
        self.latest_nav_time = 0.0
        self.latest_pick_time = 0.0
        self.base_hold = False
        self.last_status = ''

        self.create_subscription(
            Twist,
            str(self.get_parameter('nav_cmd_vel_topic').value),
            self._on_nav_cmd,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('pick_cmd_vel_topic').value),
            self._on_pick_cmd,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('mode_topic').value),
            self._on_mode,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('base_hold_topic').value),
            self._on_base_hold,
            10,
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            str(self.get_parameter('output_cmd_vel_topic').value),
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            10,
        )

        rate = float(self.get_parameter('control_rate_hz').value)
        period = 1.0 / rate if rate > 0.0 else 0.05
        self.create_timer(period, self._on_timer)
        self._publish_status(force=True)

    def _on_nav_cmd(self, msg):
        self.latest_nav_cmd = msg
        self.latest_nav_time = time.monotonic()

    def _on_pick_cmd(self, msg):
        self.latest_pick_cmd = msg
        self.latest_pick_time = time.monotonic()

    def _on_mode(self, msg):
        self.mode = self._normalize_mode(msg.data)
        self._publish_status(force=True)

    def _on_base_hold(self, msg):
        self.base_hold = bool(msg.data)
        self._publish_status(force=True)

    def _on_timer(self):
        now = time.monotonic()
        cmd = Twist()
        source = 'zero'

        if self.base_hold:
            source = 'base_hold'
        elif self.mode == 'NAV':
            if now - self.latest_nav_time <= self.nav_cmd_timeout_s:
                cmd = self.latest_nav_cmd
                source = 'nav'
            else:
                source = 'nav_timeout'
        elif self.mode == 'PICK':
            if now - self.latest_pick_time <= self.pick_cmd_timeout_s:
                cmd = self.latest_pick_cmd
                source = 'pick'
            else:
                source = 'pick_timeout'
        elif self.mode == 'HOLD':
            source = 'hold'
        elif self.mode == 'STOP':
            source = 'stop'

        self.cmd_pub.publish(cmd)
        self._publish_status(source=source)

    def _publish_status(self, source=None, force=False):
        text = (
            f'mode={self.mode} base_hold={self.base_hold} '
            f'source={source if source is not None else "none"}'
        )
        if not force and text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    @classmethod
    def _normalize_mode(cls, value):
        mode = str(value).strip().upper()
        if mode not in cls.VALID_MODES:
            return 'HOLD'
        return mode


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
