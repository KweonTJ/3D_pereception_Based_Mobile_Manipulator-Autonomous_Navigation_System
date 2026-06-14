#!/usr/bin/env python3
"""cmd_vel_mux — 리더 base 명령 state-select 중재 (twist_mux 대체, 의존성 없음).

우선순위(높→낮):
  align : /cmd_vel_align 가 timeout 내 수신 → startup 직진정렬이 base 소유.
  pnp   : /pnp/working=true 이고 /cmd_vel_pnp 신선 → 픽 중 mp_control이 base 소유.
  nav   : 그 외 → /leader/cmd_vel (평상시 rover_nav 주행).
선택 소스가 stale면 0(정지) 발행. 출력 → /cmd_vel (diff_drive 구독).

상태머신 신호(/pnp/working)를 직접 읽어 "pnp 상태면 pnp 토픽이 실제 제어로,
아니면 무시"를 구현. align은 startup 일회성이라 수신 recency로 최우선 처리.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')
        g = lambda k, v: self.declare_parameter(k, v).value
        self.out_topic = g('output_topic', '/cmd_vel')
        self.align_topic = g('align_topic', '/cmd_vel_align')
        self.pnp_topic = g('pnp_topic', '/cmd_vel_pnp')
        self.nav_topic = g('nav_topic', '/leader/cmd_vel')
        self.working_topic = g('pnp_working_topic', '/pnp/working')
        self.timeout = float(g('input_timeout_sec', 0.5))
        rate = float(g('publish_rate', 50.0))

        self.last = {'align': None, 'pnp': None, 'nav': None}  # key -> (t, Twist)
        self.pnp_working = False
        self._active = '<init>'

        self.pub = self.create_publisher(Twist, self.out_topic, 10)
        self.create_subscription(Twist, self.align_topic, lambda m: self._set('align', m), 10)
        self.create_subscription(Twist, self.pnp_topic, lambda m: self._set('pnp', m), 10)
        self.create_subscription(Twist, self.nav_topic, lambda m: self._set('nav', m), 10)
        self.create_subscription(Bool, self.working_topic,
                                 lambda m: setattr(self, 'pnp_working', bool(m.data)), 10)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'cmd_vel_mux: align[{self.align_topic}] > '
            f'pnp[{self.pnp_topic}@{self.working_topic}] > nav[{self.nav_topic}] -> {self.out_topic}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _set(self, key, msg):
        self.last[key] = (self._now(), msg)

    def _fresh(self, key):
        e = self.last[key]
        return e is not None and (self._now() - e[0]) <= self.timeout

    def _tick(self):
        if self._fresh('align'):
            src = 'align'
        elif self.pnp_working and self._fresh('pnp'):
            src = 'pnp'
        elif self._fresh('nav'):
            src = 'nav'
        else:
            src = None
        if src != self._active:
            self.get_logger().info(f'cmd_vel_mux active -> {src or "STOP(0)"}')
            self._active = src
        self.pub.publish(self.last[src][1] if src else Twist())


def main():
    rclpy.init()
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.ok() and rclpy.shutdown()


if __name__ == '__main__':
    main()
