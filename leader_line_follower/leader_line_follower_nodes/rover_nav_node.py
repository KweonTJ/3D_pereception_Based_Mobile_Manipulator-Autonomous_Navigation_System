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
from host_mission_interfaces.msg import LeaderRoute
from host_mission_interfaces.msg import NavFeedback
from host_mission_interfaces.msg import TargetCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

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
    DEFAULT_GOAL_PASS_TOLERANCE = 0.15
    DEFAULT_GOAL_SLOWDOWN_DISTANCE = 0.30
    DEFAULT_NORMAL_LINEAR_X = 0.20
    DEFAULT_MIN_LINEAR_X = 0.03
    DEFAULT_LOOKAHEAD_DISTANCE = 0.50
    DEFAULT_USE_ADAPTIVE_LOOKAHEAD = False
    DEFAULT_MIN_LOOKAHEAD_DISTANCE = 0.25
    DEFAULT_MAX_LOOKAHEAD_DISTANCE = 0.50
    DEFAULT_LOOKAHEAD_TIME = 1.0
    DEFAULT_ROTATE_TO_LOOKAHEAD_GAIN = 1.0
    DEFAULT_NEAR_GOAL_DEADBAND_SPEED = 0.02
    DEFAULT_GOAL_LOOKAHEAD_EXTENSION = 0.50
    DEFAULT_SLOWDOWN_AT_INTERMEDIATE_WAYPOINTS = True
    DEFAULT_ALIGN_START_ANGLE_DEG = 50.0
    DEFAULT_ALIGN_FINISH_ANGLE_DEG = 5.0

    def __init__(self):
        super().__init__('rover_nav_node')

        self.declare_parameter('robot', 'follower')
        self.declare_parameter('map_path', '')
        self.declare_parameter('frame_id', 'uwb_global')
        self.declare_parameter('cmd_vel_topic', '')
        self.declare_parameter('invert_angular_z', False)
        self.declare_parameter('control_rate_hz', self.DEFAULT_CONTROL_RATE_HZ)
        self.declare_parameter(
            'waypoint_tolerance', self.DEFAULT_WAYPOINT_TOLERANCE
        )
        self.declare_parameter('goal_tolerance', self.DEFAULT_GOAL_TOLERANCE)
        self.declare_parameter(
            'goal_pass_tolerance', self.DEFAULT_GOAL_PASS_TOLERANCE
        )
        self.declare_parameter(
            'goal_slowdown_distance', self.DEFAULT_GOAL_SLOWDOWN_DISTANCE
        )
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
        self.declare_parameter('normal_linear_x', self.DEFAULT_NORMAL_LINEAR_X)
        self.declare_parameter('min_linear_x', self.DEFAULT_MIN_LINEAR_X)
        self.declare_parameter('lookahead_distance', self.DEFAULT_LOOKAHEAD_DISTANCE)
        self.declare_parameter(
            'use_adaptive_lookahead', self.DEFAULT_USE_ADAPTIVE_LOOKAHEAD
        )
        self.declare_parameter(
            'min_lookahead_distance', self.DEFAULT_MIN_LOOKAHEAD_DISTANCE
        )
        self.declare_parameter(
            'max_lookahead_distance', self.DEFAULT_MAX_LOOKAHEAD_DISTANCE
        )
        self.declare_parameter('lookahead_time', self.DEFAULT_LOOKAHEAD_TIME)
        self.declare_parameter(
            'rotate_to_lookahead_gain', self.DEFAULT_ROTATE_TO_LOOKAHEAD_GAIN
        )
        self.declare_parameter(
            'near_goal_deadband_speed', self.DEFAULT_NEAR_GOAL_DEADBAND_SPEED
        )
        self.declare_parameter(
            'goal_lookahead_extension', self.DEFAULT_GOAL_LOOKAHEAD_EXTENSION
        )
        self.declare_parameter(
            'slowdown_at_intermediate_waypoints',
            self.DEFAULT_SLOWDOWN_AT_INTERMEDIATE_WAYPOINTS,
        )
        self.declare_parameter(
            'align_start_angle_deg', self.DEFAULT_ALIGN_START_ANGLE_DEG
        )
        self.declare_parameter(
            'align_finish_angle_deg', self.DEFAULT_ALIGN_FINISH_ANGLE_DEG
        )

        self.robot = str(self.get_parameter('robot').value)
        self.map_path = self.get_parameter('map_path').value
        self.frame_id = self.get_parameter('frame_id').value
        self.configured_cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.invert_angular_z = bool(self.get_parameter('invert_angular_z').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.waypoint_tolerance = float(
            self.get_parameter('waypoint_tolerance').value
        )
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.goal_pass_tolerance = float(
            self.get_parameter('goal_pass_tolerance').value
        )
        self.goal_slowdown_distance = float(
            self.get_parameter('goal_slowdown_distance').value
        )
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
        self.normal_linear_x = float(self.get_parameter('normal_linear_x').value)
        self.min_linear_x = float(self.get_parameter('min_linear_x').value)
        self.lookahead_distance = float(
            self.get_parameter('lookahead_distance').value
        )
        self.use_adaptive_lookahead = bool(
            self.get_parameter('use_adaptive_lookahead').value
        )
        self.min_lookahead_distance = float(
            self.get_parameter('min_lookahead_distance').value
        )
        self.max_lookahead_distance = float(
            self.get_parameter('max_lookahead_distance').value
        )
        self.lookahead_time = float(self.get_parameter('lookahead_time').value)
        self.rotate_to_lookahead_gain = float(
            self.get_parameter('rotate_to_lookahead_gain').value
        )
        self.near_goal_deadband_speed = float(
            self.get_parameter('near_goal_deadband_speed').value
        )
        self.goal_lookahead_extension = float(
            self.get_parameter('goal_lookahead_extension').value
        )
        self.slowdown_at_intermediate_waypoints = bool(
            self.get_parameter('slowdown_at_intermediate_waypoints').value
        )
        self.align_start_angle_deg = float(
            self.get_parameter('align_start_angle_deg').value
        )
        self.align_finish_angle_deg = float(
            self.get_parameter('align_finish_angle_deg').value
        )
        self.align_start_angle = math.radians(self.align_start_angle_deg)
        self.align_finish_angle = math.radians(self.align_finish_angle_deg)

        self.nodes = {}
        self.graph = {}
        self.map_frame_id = ''
        self._load_map()

        prefix = f'/{self.robot}'
        self.cmd_vel_topic = self.configured_cmd_vel_topic or f'{prefix}/cmd_vel'

        target_qos = QoSProfile(depth=1)
        target_qos.reliability = ReliabilityPolicy.RELIABLE
        target_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.target_sub = self.create_subscription(
            TargetCommand, f'{prefix}/target_cmd', self.target_callback, target_qos
        )
        self.position_sub = self.create_subscription(
            PoseStamped, f'{prefix}/global/position', self.position_callback, 10
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.feedback_pub = self.create_publisher(
            NavFeedback, f'{prefix}/nav_feedback', 10
        )
        self.active_waypoint_pub = self.create_publisher(
            Point, f'{prefix}/active_waypoint', 10
        )
        self.path_error_pub = self.create_publisher(
            Point, f'{prefix}/path_error', 10
        )
        # Planned route over the shared lane graph, for platooning. Latched
        # (transient_local) so a late-joining follower gets the current route;
        # QoS matches the host mission bridge (25->30->73) that relays it.
        route_qos = QoSProfile(depth=1)
        route_qos.reliability = ReliabilityPolicy.RELIABLE
        route_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.route_pub = self.create_publisher(
            LeaderRoute, f'{prefix}/route', route_qos
        )

        self.current_xy = None
        self.current_yaw = None
        self.last_pose_time = 0.0

        self.state = 'IDLE'
        self.current_cmd_id = 0
        self._planned_cmd_id = -1
        self.current_target_node = -1
        self.pending_command = None
        self.route_nodes = []
        self.waypoints = []
        self.waypoint_index = 0
        self.route_start_xy = None
        self.aligning_to_target = False
        self.stop_and_hold = True
        self.speed_cap = self.default_speed_cap
        self.command_stamp_sec = 0.0
        self.detail = 'idle'
        self.last_feedback_time = 0.0
        self.log_counter = 0

        period = 1.0 / self.control_rate_hz if self.control_rate_hz > 0.0 else 0.05
        self.timer = self.create_timer(period, self.control_callback)

        self.get_logger().info(
            f'{self.robot} rover nav ready: /{self.robot}/target_cmd + '
            f'/{self.robot}/global/position -> {self.cmd_vel_topic}'
        )
        self.get_logger().info(
            f'{self.robot} Pure Pursuit control: '
            f'lookahead={self.lookahead_distance:.2f}m, '
            f'align={self.align_start_angle_deg:.1f}/{self.align_finish_angle_deg:.1f}deg, '
            f'invert_angular_z={self.invert_angular_z}'
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
            if int(msg.cmd_id) == self._planned_cmd_id:
                return
            self._handle_goto(msg)
        elif mode == 'STOP':
            self.pending_command = None
            self._clear_route()
            self.state = 'STOPPED'
            self.detail = 'stop command'
            self._publish_stop()
            self._publish_feedback(force=True)
            self._publish_route()
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
        self._planned_cmd_id = self.current_cmd_id
        self.route_start_xy = self.current_xy
        self.aligning_to_target = False
        self.log_counter = 0
        self.state = 'TRACKING'
        self.detail = 'route=' + '->'.join(str(node_id) for node_id in route_nodes)
        self._publish_feedback(force=True)
        self._publish_active_waypoint()
        self._publish_route()

    def _publish_route(self):
        # Share the planned lane-graph route for platooning. node_ids is the
        # ordered route current->target; an empty list signals "no active route"
        # (e.g. after STOP). The follower decides where to JOIN/BRANCH off it.
        msg = LeaderRoute()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.robot = self.robot
        msg.cmd_id = self.current_cmd_id
        msg.target_node = self.current_target_node
        msg.node_ids = [int(node_id) for node_id in self.route_nodes]
        self.route_pub.publish(msg)

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
            self.aligning_to_target = False
            if self.waypoint_index >= len(self.waypoints):
                self._arrive()
                return
            waypoint = self.waypoints[self.waypoint_index]
            dist = self._distance(self.current_xy, waypoint)
            final_waypoint = self.waypoint_index >= len(self.waypoints) - 1
            tolerance = self.goal_tolerance if final_waypoint else self.waypoint_tolerance

        segment_start = self._segment_start_xy()
        path_dx = waypoint[0] - segment_start[0]
        path_dy = waypoint[1] - segment_start[1]
        path_length = math.hypot(path_dx, path_dy)
        if path_length <= 1e-6:
            self.waypoint_index += 1
            self.aligning_to_target = False
            return

        unit_x = path_dx / path_length
        unit_y = path_dy / path_length

        rel_x = self.current_xy[0] - segment_start[0]
        rel_y = self.current_xy[1] - segment_start[1]
        along_track = rel_x * unit_x + rel_y * unit_y
        cross_track = unit_x * rel_y - unit_y * rel_x

        goal_dx = waypoint[0] - self.current_xy[0]
        goal_dy = waypoint[1] - self.current_xy[1]
        distance_to_goal = math.hypot(goal_dx, goal_dy)
        remaining = path_length - along_track

        err = Point()
        err.x = float(cross_track)
        err.y = float(along_track)
        err.z = float(distance_to_goal)
        self.path_error_pub.publish(err)

        if final_waypoint and distance_to_goal <= self.goal_tolerance:
            self._arrive()
            return

        if remaining <= 0.0:
            if final_waypoint:
                self._arrive()
                return
            if distance_to_goal <= self.goal_pass_tolerance:
                self.waypoint_index += 1
                self.aligning_to_target = False
                return
            self.get_logger().warn(
                f'Passed waypoint but missed tolerance. '
                f'wp={self.waypoint_index + 1}/{len(self.waypoints)} '
                f'dist={distance_to_goal:.3f}, cross={cross_track:.3f}'
            )

        goal_bearing = math.atan2(goal_dy, goal_dx)
        goal_heading_error = self._normalize_angle(goal_bearing - self.current_yaw)

        if abs(goal_heading_error) > self.align_start_angle:
            self.aligning_to_target = True

        twist = Twist()
        mode = 'pp'
        lookahead = self._lookahead_distance()
        target_along = 0.0
        target_angle = 0.0
        curvature = 0.0
        x_l = 0.0
        y_l = 0.0

        if self.aligning_to_target and abs(goal_heading_error) > self.align_finish_angle:
            twist.linear.x = 0.0
            twist.angular.z = self._clamp(
                self.rotate_to_lookahead_gain * goal_heading_error,
                -self.max_angular_speed,
                self.max_angular_speed,
            )
            mode = 'goal-align'
        else:
            self.aligning_to_target = False

            virtual_path_length = path_length + self.goal_lookahead_extension
            target_along = along_track + lookahead
            target_along = max(0.0, min(target_along, virtual_path_length))

            lookahead_x = segment_start[0] + target_along * unit_x
            lookahead_y = segment_start[1] + target_along * unit_y

            dx_l = lookahead_x - self.current_xy[0]
            dy_l = lookahead_y - self.current_xy[1]

            cos_t = math.cos(self.current_yaw)
            sin_t = math.sin(self.current_yaw)
            x_l = cos_t * dx_l + sin_t * dy_l
            y_l = -sin_t * dx_l + cos_t * dy_l

            ld2 = x_l * x_l + y_l * y_l
            if ld2 > 1e-6:
                curvature = 2.0 * y_l / ld2

            v = min(self.normal_linear_x, self.speed_cap, self.max_linear_speed)
            should_slow_for_waypoint = (
                final_waypoint or self.slowdown_at_intermediate_waypoints
            )
            if should_slow_for_waypoint and distance_to_goal < self.goal_slowdown_distance:
                slowdown = distance_to_goal / self.goal_slowdown_distance
                slowdown = max(0.0, min(slowdown, 1.0))
                v *= slowdown
                if v < self.near_goal_deadband_speed:
                    v = 0.0
            else:
                v = max(v, self.min_linear_x)
                v = min(v, self.speed_cap, self.max_linear_speed)

            target_angle = math.atan2(y_l, x_l)
            if x_l < 0.0:
                v = 0.0
                w = self.rotate_to_lookahead_gain * target_angle
                mode = 'pp-rotate'
            else:
                if abs(curvature) > 1e-6:
                    v = min(v, self.max_angular_speed / abs(curvature))
                w = v * curvature

            twist.linear.x = float(v)
            twist.angular.z = float(
                self._clamp(w, -self.max_angular_speed, self.max_angular_speed)
            )

        if self.invert_angular_z:
            twist.angular.z *= -1.0
        self.cmd_pub.publish(twist)
        self._publish_active_waypoint()

        self.detail = (
            f'wp={self.waypoint_index + 1}/{len(self.waypoints)} '
            f'target_node={self.current_target_node} {mode} '
            f'cross={cross_track:.3f}m along={along_track:.3f}m '
            f'dist={distance_to_goal:.3f}m'
        )
        self._publish_feedback()

        self.log_counter += 1
        if self.log_counter % 15 == 0:
            self.get_logger().info(
                f'{mode} | cross={cross_track:.3f}m '
                f'along={along_track:.3f}m rem={remaining:.3f}m '
                f'dist={distance_to_goal:.3f}m '
                f'goal_ang={math.degrees(goal_heading_error):.1f}deg '
                f'Ld={lookahead:.2f} xL={x_l:.3f} yL={y_l:.3f} '
                f'target_along={target_along:.3f}m '
                f'target_ang={math.degrees(target_angle):.1f}deg '
                f'curv={curvature:.3f} vx={twist.linear.x:.2f} '
                f'wz={twist.angular.z:.2f}'
            )

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
        self.route_start_xy = None
        self.aligning_to_target = False

    def _segment_start_xy(self):
        if self.waypoint_index <= 0:
            return self.route_start_xy or self.current_xy
        return self.waypoints[self.waypoint_index - 1]

    def _lookahead_distance(self):
        base_v = min(self.normal_linear_x, self.speed_cap, self.max_linear_speed)
        if self.use_adaptive_lookahead:
            lookahead = self.lookahead_time * abs(base_v)
            lookahead = max(self.min_lookahead_distance, lookahead)
            return min(self.max_lookahead_distance, lookahead)
        return self.lookahead_distance

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
