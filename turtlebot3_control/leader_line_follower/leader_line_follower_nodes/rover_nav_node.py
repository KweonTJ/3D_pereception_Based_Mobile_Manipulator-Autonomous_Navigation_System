#!/usr/bin/env python3
import heapq
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from host_mission_interfaces.msg import NavFeedback
from host_mission_interfaces.msg import TargetCommand
from rclpy.node import Node

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


class RoverNavNode(Node):
    """Namespaced rover navigation node.

    The mission boundary is /{robot}/target_cmd. This node owns graph routing,
    waypoint tracking, and /{robot}/cmd_vel output; it intentionally does not
    use legacy shared topics such as /map_goal, /target_point, or /cmd_vel.
    """

    DEFAULT_CONTROL_RATE_HZ = 20.0
    DEFAULT_WAYPOINT_TOLERANCE = 0.12
    DEFAULT_GOAL_TOLERANCE = 0.12
    DEFAULT_MAX_LINEAR_SPEED = 0.15
    DEFAULT_MAX_ANGULAR_SPEED = 0.8
    DEFAULT_ANGULAR_KP = 1.5
    DEFAULT_HEADING_THRESHOLD = 0.8
    DEFAULT_COMMAND_TIMEOUT_SEC = 0.0
    DEFAULT_SPEED_CAP = 0.12
    DEFAULT_FEEDBACK_PERIOD_SEC = 0.25

    def __init__(self):
        super().__init__('rover_nav_node')

        self.declare_parameter('robot', 'follower')
        self.declare_parameter('map_path', '')
        self.declare_parameter('frame_id', 'uwb_global')
        self.declare_parameter('control_rate_hz', self.DEFAULT_CONTROL_RATE_HZ)
        self.declare_parameter(
            'waypoint_tolerance', self.DEFAULT_WAYPOINT_TOLERANCE
        )
        self.declare_parameter('goal_tolerance', self.DEFAULT_GOAL_TOLERANCE)
        self.declare_parameter('max_linear_speed', self.DEFAULT_MAX_LINEAR_SPEED)
        self.declare_parameter('max_angular_speed', self.DEFAULT_MAX_ANGULAR_SPEED)
        self.declare_parameter('angular_kp', self.DEFAULT_ANGULAR_KP)
        self.declare_parameter('heading_threshold', self.DEFAULT_HEADING_THRESHOLD)
        self.declare_parameter(
            'command_timeout_sec', self.DEFAULT_COMMAND_TIMEOUT_SEC
        )
        self.declare_parameter('default_speed_cap', self.DEFAULT_SPEED_CAP)
        self.declare_parameter(
            'feedback_period_sec', self.DEFAULT_FEEDBACK_PERIOD_SEC
        )

        self.robot = str(self.get_parameter('robot').value)
        self.map_path = self.get_parameter('map_path').value
        self.frame_id = self.get_parameter('frame_id').value
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.waypoint_tolerance = float(
            self.get_parameter('waypoint_tolerance').value
        )
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.angular_kp = float(self.get_parameter('angular_kp').value)
        self.heading_threshold = float(self.get_parameter('heading_threshold').value)
        self.command_timeout_sec = float(
            self.get_parameter('command_timeout_sec').value
        )
        self.default_speed_cap = float(self.get_parameter('default_speed_cap').value)
        self.feedback_period_sec = float(
            self.get_parameter('feedback_period_sec').value
        )

        self.nodes = {}
        self.graph = {}
        self.map_frame_id = ''
        self._load_map()

        prefix = f'/{self.robot}'
        self.target_sub = self.create_subscription(
            TargetCommand, f'{prefix}/target_cmd', self.target_callback, 10
        )
        self.position_sub = self.create_subscription(
            PoseStamped, f'{prefix}/global/position', self.position_callback, 10
        )

        self.cmd_pub = self.create_publisher(Twist, f'{prefix}/cmd_vel', 10)
        self.feedback_pub = self.create_publisher(
            NavFeedback, f'{prefix}/nav_feedback', 10
        )
        self.active_waypoint_pub = self.create_publisher(
            Point, f'{prefix}/active_waypoint', 10
        )
        self.path_error_pub = self.create_publisher(
            Point, f'{prefix}/path_error', 10
        )

        self.current_xy = None
        self.current_yaw = None
        self.last_pose_time = 0.0

        self.state = 'IDLE'
        self.current_cmd_id = 0
        self.current_target_node = -1
        self.pending_command = None
        self.route_nodes = []
        self.waypoints = []
        self.waypoint_index = 0
        self.stop_and_hold = True
        self.speed_cap = self.default_speed_cap
        self.command_stamp_sec = 0.0
        self.detail = 'idle'
        self.last_feedback_time = 0.0

        period = 1.0 / self.control_rate_hz if self.control_rate_hz > 0.0 else 0.05
        self.timer = self.create_timer(period, self.control_callback)

        self.get_logger().info(
            f'{self.robot} rover nav ready: /{self.robot}/target_cmd + '
            f'/{self.robot}/global/position -> /{self.robot}/cmd_vel'
        )

    def position_callback(self, msg):
        pos = msg.pose.position
        ori = msg.pose.orientation
        self.current_xy = (float(pos.x), float(pos.y))
        self.current_yaw = self._yaw_from_quaternion(
            ori.x, ori.y, ori.z, ori.w
        )
        self.last_pose_time = time.time()

        if self.pending_command is not None:
            pending = self.pending_command
            self.pending_command = None
            self._handle_goto(pending)

    def target_callback(self, msg):
        if msg.robot and msg.robot != self.robot:
            return

        if int(msg.cmd_id) < self.current_cmd_id:
            self.get_logger().warn(
                f'Ignoring stale command cmd_id={msg.cmd_id}; '
                f'current={self.current_cmd_id}'
            )
            return

        mode = msg.mode.strip().upper()
        self.current_cmd_id = int(msg.cmd_id)
        self.command_stamp_sec = self._stamp_to_sec(msg.header.stamp)

        if mode == 'GOTO':
            self._handle_goto(msg)
        elif mode == 'STOP':
            self.pending_command = None
            self._clear_route()
            self.state = 'STOPPED'
            self.detail = 'stop command'
            self._publish_stop()
            self._publish_feedback(force=True)
        elif mode == 'HOLD':
            self.pending_command = None
            self.state = 'HOLDING'
            self.detail = 'hold command'
            self._publish_stop()
            self._publish_feedback(force=True)
        elif mode == 'RELEASE':
            self.pending_command = None
            if self.waypoints and self.waypoint_index < len(self.waypoints):
                self.state = 'TRACKING'
                self.detail = 'released; tracking resumed'
            else:
                self.state = 'IDLE'
                self.detail = 'released; no active route'
            self._publish_feedback(force=True)
        else:
            self._error(f'unsupported mode: {mode}')

    def _handle_goto(self, msg):
        self.current_target_node = int(msg.target_node)
        self.stop_and_hold = bool(msg.stop_and_hold)
        self.speed_cap = (
            float(msg.speed_cap)
            if float(msg.speed_cap) > 0.0
            else self.default_speed_cap
        )
        self.speed_cap = min(self.speed_cap, self.max_linear_speed)

        if self.current_xy is None:
            self.pending_command = msg
            self.state = 'ROUTING'
            self.detail = 'waiting for position'
            self._publish_stop()
            self._publish_feedback(force=True)
            return

        if self.current_target_node >= 0:
            if self.current_target_node not in self.nodes:
                self._error('target node not found')
                return
            goal_node = self.current_target_node
            exact_goal = None
        else:
            target = (float(msg.target_coord.x), float(msg.target_coord.y))
            goal_node, _ = self._nearest_node(target)
            exact_goal = target

        start_node, _ = self._nearest_node(self.current_xy)
        self.state = 'ROUTING'
        self.detail = f'routing {start_node}->{goal_node}'
        self._publish_feedback(force=True)

        route_nodes = self._dijkstra(start_node, goal_node)
        if not route_nodes:
            self._error(f'no route from {start_node} to {goal_node}')
            return

        waypoints = [self.nodes[node_id] for node_id in route_nodes]
        if exact_goal is not None and self._distance(waypoints[-1], exact_goal) > 0.03:
            waypoints.append(exact_goal)

        self.route_nodes = route_nodes
        self.waypoints = waypoints
        self.waypoint_index = 0
        self.state = 'TRACKING'
        self.detail = 'route=' + '->'.join(str(node_id) for node_id in route_nodes)
        self._publish_feedback(force=True)
        self._publish_active_waypoint()

    def control_callback(self):
        if self.command_timeout_sec > 0.0 and self.command_stamp_sec > 0.0:
            if time.time() - self.command_stamp_sec > self.command_timeout_sec:
                if self.state not in ('STOPPED', 'ERROR'):
                    self._error('command timeout')
                return

        if self.state in ('IDLE', 'STOPPED', 'HOLDING', 'ERROR'):
            self._publish_stop()
            self._publish_feedback()
            return

        if self.state != 'TRACKING':
            self._publish_feedback()
            return

        if self.current_xy is None or self.current_yaw is None:
            self._publish_stop()
            self.detail = 'waiting for position'
            self._publish_feedback()
            return

        if self.waypoint_index >= len(self.waypoints):
            self._arrive()
            return

        waypoint = self.waypoints[self.waypoint_index]
        dist = self._distance(self.current_xy, waypoint)
        final_waypoint = self.waypoint_index >= len(self.waypoints) - 1
        tolerance = self.goal_tolerance if final_waypoint else self.waypoint_tolerance

        while dist <= tolerance:
            self.waypoint_index += 1
            if self.waypoint_index >= len(self.waypoints):
                self._arrive()
                return
            waypoint = self.waypoints[self.waypoint_index]
            dist = self._distance(self.current_xy, waypoint)
            final_waypoint = self.waypoint_index >= len(self.waypoints) - 1
            tolerance = self.goal_tolerance if final_waypoint else self.waypoint_tolerance

        dx = waypoint[0] - self.current_xy[0]
        dy = waypoint[1] - self.current_xy[1]
        desired_yaw = math.atan2(dy, dx)
        yaw_error = self._normalize_angle(desired_yaw - self.current_yaw)

        angular_z = self._clamp(
            self.angular_kp * yaw_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        linear_x = min(self.speed_cap, self.max_linear_speed)
        if abs(yaw_error) > self.heading_threshold:
            linear_x = 0.0
        elif final_waypoint:
            linear_x *= max(0.25, min(1.0, dist / 0.35))

        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_pub.publish(twist)

        err = Point()
        err.x = float(yaw_error)
        err.y = float(dist)
        err.z = float(self.waypoint_index)
        self.path_error_pub.publish(err)
        self._publish_active_waypoint()

        self.detail = (
            f'wp={self.waypoint_index + 1}/{len(self.waypoints)} '
            f'target_node={self.current_target_node}'
        )
        self._publish_feedback()

    def _arrive(self):
        self._publish_stop()
        self.waypoint_index = len(self.waypoints)
        self.detail = f'arrived target_node={self.current_target_node}'
        self.state = 'ARRIVED'
        self._publish_feedback(force=True)
        if self.stop_and_hold:
            self.state = 'HOLDING'
            self.detail = f'holding target_node={self.current_target_node}'
        else:
            self._clear_route()
            self.state = 'IDLE'
            self.detail = 'idle after arrival'
        self._publish_feedback(force=True)

    def _error(self, detail):
        self._clear_route()
        self.pending_command = None
        self.state = 'ERROR'
        self.detail = detail
        self._publish_stop()
        self._publish_feedback(force=True)
        self.get_logger().error(f'{self.robot}: {detail}')

    def _clear_route(self):
        self.route_nodes = []
        self.waypoints = []
        self.waypoint_index = 0

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    def _publish_active_waypoint(self):
        if not self.waypoints or self.waypoint_index >= len(self.waypoints):
            return
        waypoint = self.waypoints[self.waypoint_index]
        msg = Point()
        msg.x = float(waypoint[0])
        msg.y = float(waypoint[1])
        msg.z = 0.0
        self.active_waypoint_pub.publish(msg)

    def _publish_feedback(self, force=False):
        now = time.time()
        if (
            not force
            and self.feedback_period_sec > 0.0
            and now - self.last_feedback_time < self.feedback_period_sec
        ):
            return
        self.last_feedback_time = now

        msg = NavFeedback()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.robot = self.robot
        msg.cmd_id = int(self.current_cmd_id)
        msg.state = self.state
        msg.current_node = self._current_nearest_node()
        msg.progress = self._progress()
        msg.detail = self.detail
        self.feedback_pub.publish(msg)

    def _current_nearest_node(self):
        if self.current_xy is None or not self.nodes:
            return -1
        node_id, _ = self._nearest_node(self.current_xy)
        return int(node_id)

    def _progress(self):
        if not self.waypoints:
            return 1.0 if self.state in ('ARRIVED', 'HOLDING') else 0.0
        return max(0.0, min(1.0, self.waypoint_index / float(len(self.waypoints))))

    def _load_map(self):
        map_path = self._resolve_map_path(self.map_path)
        with map_path.open('r', encoding='utf-8') as map_file:
            data = json.load(map_file)

        self.map_frame_id = data.get('frame_id', self.frame_id)
        graph = data['graph']
        self.nodes = {
            int(node['id']): (float(node['x']), float(node['y']))
            for node in graph['nodes']
        }
        self.graph = {node_id: [] for node_id in self.nodes}

        for edge in graph['edges']:
            start = int(edge['from'])
            end = int(edge['to'])
            if 'cost' in edge:
                cost = float(edge['cost'])
            else:
                cost = self._distance(self.nodes[start], self.nodes[end])
            self.graph[start].append((end, cost))
            if not bool(edge.get('directed', False)):
                self.graph[end].append((start, cost))

        self.get_logger().info(
            f'Loaded rover map: {map_path}, frame={self.map_frame_id}, '
            f'nodes={len(self.nodes)}'
        )

    def _resolve_map_path(self, configured_path):
        candidates = []
        if configured_path:
            candidates.append(Path(configured_path).expanduser())

        if get_package_share_directory is not None:
            try:
                share = Path(get_package_share_directory('leader_line_follower'))
                candidates.append(share / 'maps' / 'map.json')
            except Exception:
                pass

        candidates.append(Path.cwd() / 'host_planning_ref' / 'map.json')
        candidates.append(
            Path(__file__).resolve().parents[3] / 'host_planning_ref' / 'map.json'
        )

        for candidate in candidates:
            if candidate.is_file():
                return candidate

        searched = ', '.join(str(candidate) for candidate in candidates)
        raise FileNotFoundError(f'Could not find map.json. Searched: {searched}')

    def _nearest_node(self, xy):
        best_node = None
        best_dist = float('inf')
        for node_id, node_xy in self.nodes.items():
            dist = self._distance(xy, node_xy)
            if dist < best_dist:
                best_node = node_id
                best_dist = dist
        return best_node, best_dist

    def _dijkstra(self, start, goal):
        queue = [(0.0, start)]
        distances = {start: 0.0}
        parents = {start: None}

        while queue:
            cost, node = heapq.heappop(queue)
            if cost > distances[node]:
                continue
            if node == goal:
                break
            for neighbor, edge_cost in self.graph.get(node, []):
                new_cost = cost + edge_cost
                if new_cost < distances.get(neighbor, float('inf')):
                    distances[neighbor] = new_cost
                    parents[neighbor] = node
                    heapq.heappush(queue, (new_cost, neighbor))

        if goal not in parents:
            return []

        path = []
        node = goal
        while node is not None:
            path.append(node)
            node = parents[node]
        path.reverse()
        return path

    @staticmethod
    def _distance(a, b):
        return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))

    @staticmethod
    def _yaw_from_quaternion(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))

    @staticmethod
    def _stamp_to_sec(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = RoverNavNode()
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
