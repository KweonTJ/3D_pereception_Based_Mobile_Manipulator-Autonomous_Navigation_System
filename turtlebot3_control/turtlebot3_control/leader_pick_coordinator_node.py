#!/usr/bin/env python3
import time

import rclpy
from host_mission_interfaces.msg import NavFeedback
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String


class LeaderPickCoordinatorNode(Node):
    """Gate ArUco pick-place until rover navigation has actually arrived."""

    def __init__(self):
        super().__init__('leader_pick_coordinator_node')

        self.declare_parameter('nav_feedback_topic', '/leader/nav_feedback')
        self.declare_parameter('aruco_visible_topic', '/target/aruco_visible')
        self.declare_parameter('mp_control_status_topic', '/mp_control/status')
        self.declare_parameter('mp_control_start_topic', '/mp_control/start')
        self.declare_parameter('mux_mode_topic', '/turtlebot3_control/mux_mode')
        self.declare_parameter('status_topic', '/turtlebot3_control/coordinator_status')
        self.declare_parameter('wait_for_nav_arrival', True)
        self.declare_parameter('require_aruco_visible', True)
        self.declare_parameter('arrived_states', ['ARRIVED', 'HOLDING'])
        self.declare_parameter('start_publish_count', 3)
        self.declare_parameter('start_publish_period_s', 0.2)
        self.declare_parameter('aruco_visible_timeout_s', 0.8)
        self.declare_parameter('nav_feedback_timeout_s', 2.0)
        self.declare_parameter('done_status_keywords', [
            'handoff release complete',
            'handoff stay complete',
            'pick place complete',
            'grasp complete',
        ])
        self.declare_parameter('error_status_keywords', [
            'abort',
            'ERROR',
            'failed',
        ])

        self.wait_for_nav_arrival = bool(
            self.get_parameter('wait_for_nav_arrival').value)
        self.require_aruco_visible = bool(
            self.get_parameter('require_aruco_visible').value)
        self.arrived_states = {
            str(state).strip().upper()
            for state in self.get_parameter('arrived_states').value
        }
        self.start_publish_count = int(self.get_parameter('start_publish_count').value)
        self.start_publish_period_s = float(
            self.get_parameter('start_publish_period_s').value)
        self.aruco_visible_timeout_s = float(
            self.get_parameter('aruco_visible_timeout_s').value)
        self.nav_feedback_timeout_s = float(
            self.get_parameter('nav_feedback_timeout_s').value)
        self.done_status_keywords = [
            str(word) for word in self.get_parameter('done_status_keywords').value
        ]
        self.error_status_keywords = [
            str(word) for word in self.get_parameter('error_status_keywords').value
        ]

        self.nav_state = 'UNKNOWN'
        self.nav_feedback_time = 0.0
        self.aruco_visible = False
        self.aruco_visible_time = 0.0
        self.mp_status = ''
        self.phase = 'NAVIGATING'
        self.start_sent = 0
        self.last_start_time = 0.0
        self.last_status = ''

        self.create_subscription(
            NavFeedback,
            str(self.get_parameter('nav_feedback_topic').value),
            self._on_nav_feedback,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('aruco_visible_topic').value),
            self._on_aruco_visible,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('mp_control_status_topic').value),
            self._on_mp_status,
            10,
        )

        self.start_pub = self.create_publisher(
            Bool,
            str(self.get_parameter('mp_control_start_topic').value),
            10,
        )
        self.mode_pub = self.create_publisher(
            String,
            str(self.get_parameter('mux_mode_topic').value),
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            10,
        )

        self.create_timer(0.05, self._on_timer)

    def _on_nav_feedback(self, msg):
        self.nav_state = str(msg.state).strip().upper()
        self.nav_feedback_time = time.monotonic()

    def _on_aruco_visible(self, msg):
        self.aruco_visible = bool(msg.data)
        if self.aruco_visible:
            self.aruco_visible_time = time.monotonic()

    def _on_mp_status(self, msg):
        self.mp_status = msg.data

    def _on_timer(self):
        now = time.monotonic()

        nav_fresh = (
            self.nav_feedback_time > 0.0 and
            now - self.nav_feedback_time <= self.nav_feedback_timeout_s
        )
        aruco_fresh = (
            self.aruco_visible and
            self.aruco_visible_time > 0.0 and
            now - self.aruco_visible_time <= self.aruco_visible_timeout_s
        )

        if self.phase == 'NAVIGATING':
            self._publish_mode('NAV')
            arrived = (
                not self.wait_for_nav_arrival or
                (nav_fresh and self.nav_state in self.arrived_states)
            )
            visible_ok = (not self.require_aruco_visible) or aruco_fresh
            if arrived and visible_ok:
                self.phase = 'PICKING'
                self.start_sent = 0
                self.last_start_time = 0.0
                self._publish_status(force=True)

        elif self.phase == 'PICKING':
            self._publish_mode('PICK')
            self._maybe_publish_start(now)
            status_lower = self.mp_status.lower()
            if any(word.lower() in status_lower for word in self.done_status_keywords):
                self.phase = 'DONE'
                self._publish_status(force=True)
            elif any(word.lower() in status_lower for word in self.error_status_keywords):
                self.phase = 'ERROR'
                self._publish_status(force=True)

        elif self.phase in ('DONE', 'ERROR'):
            self._publish_mode('HOLD')

        self._publish_status(
            nav_fresh=nav_fresh,
            aruco_fresh=aruco_fresh,
        )

    def _maybe_publish_start(self, now):
        if self.start_publish_count >= 0 and self.start_sent >= self.start_publish_count:
            return
        if (
            self.start_sent > 0 and
            now - self.last_start_time < self.start_publish_period_s
        ):
            return
        msg = Bool()
        msg.data = True
        self.start_pub.publish(msg)
        self.start_sent += 1
        self.last_start_time = now

    def _publish_mode(self, mode):
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)

    def _publish_status(self, nav_fresh=None, aruco_fresh=None, force=False):
        text = (
            f'phase={self.phase} nav_state={self.nav_state} '
            f'nav_fresh={nav_fresh} aruco_visible={self.aruco_visible} '
            f'aruco_fresh={aruco_fresh} start_sent={self.start_sent}'
        )
        if not force and text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderPickCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
