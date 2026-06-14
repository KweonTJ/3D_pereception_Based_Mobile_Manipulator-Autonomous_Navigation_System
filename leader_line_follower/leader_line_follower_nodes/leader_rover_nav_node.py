#!/usr/bin/env python3
import rclpy

from leader_line_follower_nodes.rover_nav_node import RoverNavNode


class LeaderRoverNavNode(RoverNavNode):
    """Leader rover navigation using Pure Pursuit with conservative defaults.

    The leader robot has the same differential-drive wheel layout, but the URDF
    shows extra upper mass from the manipulator, camera plate, and cameras. The
    controller therefore keeps the follower Pure Pursuit waypoint logic while
    using lower speed and angular limits by default.
    """

    DEFAULT_MAX_LINEAR_SPEED = 0.10
    DEFAULT_MAX_ANGULAR_SPEED = 0.45
    DEFAULT_ANGULAR_KP = 1.0
    DEFAULT_HEADING_THRESHOLD = 0.55
    DEFAULT_SPEED_CAP = 0.08
    DEFAULT_NORMAL_LINEAR_X = 0.08
    DEFAULT_MIN_LINEAR_X = 0.03
    DEFAULT_GOAL_TOLERANCE = 0.14
    DEFAULT_WAYPOINT_TOLERANCE = 0.14
    DEFAULT_GOAL_PASS_TOLERANCE = 0.15
    DEFAULT_GOAL_SLOWDOWN_DISTANCE = 0.30
    DEFAULT_LOOKAHEAD_DISTANCE = 0.50
    DEFAULT_NEAR_GOAL_DEADBAND_SPEED = 0.02
    DEFAULT_GOAL_LOOKAHEAD_EXTENSION = 0.50
    DEFAULT_SLOWDOWN_AT_INTERMEDIATE_WAYPOINTS = True
    DEFAULT_ALIGN_START_ANGLE_DEG = 50.0
    DEFAULT_ALIGN_FINISH_ANGLE_DEG = 5.0

    def __init__(self):
        super().__init__()
        self.get_logger().info(
            f'{self.robot} leader Pure Pursuit controller ready: '
            f'max_v={self.max_linear_speed:.2f}m/s, '
            f'max_w={self.max_angular_speed:.2f}rad/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LeaderRoverNavNode()
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
