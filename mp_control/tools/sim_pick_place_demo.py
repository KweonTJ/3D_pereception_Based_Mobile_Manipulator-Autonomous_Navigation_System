#!/usr/bin/env python3
"""RViz-focused simulated pick-and-place demonstration.

This node is intentionally a visualization/demo sequencer. It publishes the
same bbox used by the tracker, commands the Gazebo arm controller through a
joint trajectory, and publishes RViz markers so the object is visible while it
is picked and placed.
"""

import math
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformListener
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

try:
    from ros_gz_interfaces.msg import Entity
    from ros_gz_interfaces.srv import SetEntityPose
except ImportError:
    Entity = None
    SetEntityPose = None


def duration(seconds):
    msg = Duration()
    whole = int(seconds)
    msg.sec = whole
    msg.nanosec = int((seconds - whole) * 1_000_000_000)
    return msg


class SimPickPlaceDemo(Node):
    def __init__(self):
        super().__init__("sim_pick_place_demo")
        self.declare_parameter("bbox", [264.0, 91.0, 112.0, 146.0])
        self.declare_parameter("bbox_topic", "/target/init_bbox")
        self.declare_parameter("eef_bbox", [280.0, 180.0, 90.0, 120.0])
        self.declare_parameter("eef_bbox_topic", "/target/eef_ibvs_bbox")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("trajectory_topic", "/arm_controller/joint_trajectory")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("marker_topic", "/mp_control/pick_place_markers")
        self.declare_parameter("status_topic", "/mp_control/pick_place_status")
        self.declare_parameter("cargo_event_topic", "/cargo/events")
        self.declare_parameter("cargo_current_id_topic", "/cargo/current_id")
        self.declare_parameter("cargo_id_prefix", "PKG")
        self.declare_parameter("cargo_sequence_start", 1)
        self.declare_parameter("gripper_action_name", "/gripper_controller/gripper_cmd")
        self.declare_parameter("gripper_joint_lower_m", -0.010)
        self.declare_parameter("gripper_joint_upper_m", 0.019)
        self.declare_parameter("gripper_finger_home_half_gap_m", 0.021)
        self.declare_parameter("gripper_pre_grasp_clearance_m", 0.012)
        self.declare_parameter("gripper_grasp_compression_m", 0.002)
        self.declare_parameter("start_delay_s", 4.0)
        self.declare_parameter("object_frame", "odom")
        self.declare_parameter("place_frame", "odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_demo_base_tf", True)
        self.declare_parameter("publish_demo_joint_states", True)
        self.declare_parameter("return_to_stow", True)
        self.declare_parameter("base_approach_distance_m", 0.30)
        self.declare_parameter("base_approach_speed_mps", 0.12)
        self.declare_parameter("base_transport_distance_m", 1.00)
        self.declare_parameter("base_transport_speed_mps", 0.12)
        self.declare_parameter("base_turn_angle_rad", 1.5708)
        self.declare_parameter("base_turn_speed_radps", 0.45)
        self.declare_parameter("post_place_move_distance_m", 1.00)
        self.declare_parameter("post_place_move_speed_mps", 0.12)
        self.declare_parameter("post_place_reverse_distance_m", 0.35)
        self.declare_parameter("post_place_reverse_speed_mps", 0.10)
        self.declare_parameter("post_place_turn_angle_rad", 1.5708)
        self.declare_parameter("post_place_turn_speed_radps", 0.45)
        self.declare_parameter("cmd_vel_wait_timeout_s", 20.0)
        self.declare_parameter("trajectory_wait_timeout_s", 20.0)
        self.declare_parameter("gripper_wait_timeout_s", 20.0)
        self.declare_parameter("require_cmd_vel_subscriber", False)
        self.declare_parameter("require_trajectory_subscriber", False)
        self.declare_parameter("require_gripper_action_server", False)
        self.declare_parameter("object_size_xyz", [0.06, 0.06, 0.10])
        self.declare_parameter("attached_object_offset_xyz", [-0.02, 0.0, 0.0])
        self.declare_parameter("grasp_accuracy_tolerance_m", 0.03)
        self.declare_parameter("place_on_follower", False)
        self.declare_parameter("direct_place_on_follower", False)
        self.declare_parameter("follower_place_frame", "follower_base_footprint")
        self.declare_parameter("follower_place_xyz", [0.0, 0.0, 0.12])
        self.declare_parameter("follower_place_z_m", 0.12)
        self.declare_parameter("follower_handoff_wait_s", 0.0)
        self.declare_parameter("sync_gazebo_object", True)
        self.declare_parameter("gazebo_set_pose_service", "/world/default/set_pose")
        self.declare_parameter("gazebo_object_entity_name", "grasp_test_cube")
        self.declare_parameter("gazebo_world_origin_xyz", [-2.12, -0.5, 0.0])
        self.declare_parameter("gazebo_pose_update_period_s", 0.10)
        self.declare_parameter("gazebo_pose_wait_timeout_s", 8.0)
        self.declare_parameter("triangulation_extend_joint_positions", [0.0, 0.65, -0.85, -1.20])
        self.declare_parameter("triangulation_extend_current_start_duration_s", 0.15)
        self.declare_parameter("triangulation_extend_joint_duration_s", 1.2)
        self.declare_parameter("triangulation_extend_joint_settle_s", 0.5)
        self.declare_parameter("pregrasp_trajectory_transform_enabled", True)
        self.declare_parameter("pregrasp_trajectory_use_pitch_blend", False)
        self.declare_parameter("pregrasp_trajectory_target_pitch_rad", 0.0)
        self.declare_parameter("pregrasp_trajectory_pitch_blend_weight", 1.0)
        self.declare_parameter("pregrasp_trajectory_joint_scales", [1.0, 1.0, -1.0, 1.0])
        self.declare_parameter("pregrasp_trajectory_joint_offsets", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("pregrasp_trajectory_joint_min_positions", [-3.14, -1.79, -0.94, -1.79])
        self.declare_parameter("pregrasp_trajectory_joint_max_positions", [3.14, 1.57, 1.38, 2.04])

        self.bbox = [float(v) for v in self.get_parameter("bbox").value]
        self.eef_bbox = [float(v) for v in self.get_parameter("eef_bbox").value]
        self.object_frame = str(self.get_parameter("object_frame").value)
        self.place_frame = str(self.get_parameter("place_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_demo_base_tf = bool(self.get_parameter("publish_demo_base_tf").value)
        self.publish_demo_joint_states = bool(
            self.get_parameter("publish_demo_joint_states").value)
        self.object_size_xyz = [
            float(v) for v in self.get_parameter("object_size_xyz").value]
        self.attached_object_offset_xyz = [
            float(v) for v in self.get_parameter("attached_object_offset_xyz").value]
        self.place_on_follower = bool(self.get_parameter("place_on_follower").value)
        self.direct_place_on_follower = (
            bool(self.get_parameter("direct_place_on_follower").value)
            and self.place_on_follower
        )
        self.follower_place_frame = str(self.get_parameter("follower_place_frame").value)
        self.follower_place_xyz = [
            float(v) for v in self.get_parameter("follower_place_xyz").value]
        self.follower_place_xyz[2] = float(
            self.get_parameter("follower_place_z_m").value)
        self.follower_handoff_wait_s = max(
            0.0, float(self.get_parameter("follower_handoff_wait_s").value))
        self.gazebo_world_origin_xyz = [
            float(v) for v in self.get_parameter("gazebo_world_origin_xyz").value]
        self.gripper_joint_lower_m = float(
            self.get_parameter("gripper_joint_lower_m").value)
        self.gripper_joint_upper_m = float(
            self.get_parameter("gripper_joint_upper_m").value)
        if self.gripper_joint_lower_m > self.gripper_joint_upper_m:
            self.gripper_joint_lower_m, self.gripper_joint_upper_m = (
                self.gripper_joint_upper_m, self.gripper_joint_lower_m)
        self.gripper_finger_home_half_gap_m = max(
            0.0, float(self.get_parameter("gripper_finger_home_half_gap_m").value))
        self.gripper_pre_grasp_clearance_m = max(
            0.0, float(self.get_parameter("gripper_pre_grasp_clearance_m").value))
        self.gripper_grasp_compression_m = max(
            0.0, float(self.get_parameter("gripper_grasp_compression_m").value))
        self.grasp_accuracy_tolerance_m = max(
            0.001, float(self.get_parameter("grasp_accuracy_tolerance_m").value))
        self.stay_arm_positions = [0.104311, 0.027612, -0.001534, -1.638291]
        self.triangulation_extend_joint_positions = self._float_list_parameter(
            "triangulation_extend_joint_positions", [0.0, 0.65, -0.85, -1.20], 4)
        self.triangulation_extend_current_start_duration_s = max(
            0.0,
            float(self.get_parameter("triangulation_extend_current_start_duration_s").value))
        self.triangulation_extend_joint_duration_s = max(
            0.2, float(self.get_parameter("triangulation_extend_joint_duration_s").value))
        self.triangulation_extend_joint_settle_s = max(
            0.0, float(self.get_parameter("triangulation_extend_joint_settle_s").value))
        self.pregrasp_trajectory_transform_enabled = bool(
            self.get_parameter("pregrasp_trajectory_transform_enabled").value)
        self.pregrasp_trajectory_use_pitch_blend = bool(
            self.get_parameter("pregrasp_trajectory_use_pitch_blend").value)
        self.pregrasp_trajectory_target_pitch_rad = float(
            self.get_parameter("pregrasp_trajectory_target_pitch_rad").value)
        self.pregrasp_trajectory_pitch_blend_weight = min(
            max(float(self.get_parameter("pregrasp_trajectory_pitch_blend_weight").value), 0.0),
            1.0,
        )
        self.pregrasp_trajectory_joint_scales = self._float_list_parameter(
            "pregrasp_trajectory_joint_scales", [1.0, 1.0, -1.0, 1.0], 4)
        self.pregrasp_trajectory_joint_offsets = self._float_list_parameter(
            "pregrasp_trajectory_joint_offsets", [0.0, 0.0, 0.0, 0.0], 4)
        self.pregrasp_trajectory_joint_min_positions = self._float_list_parameter(
            "pregrasp_trajectory_joint_min_positions", [-3.14, -1.79, -0.94, -1.79], 4)
        self.pregrasp_trajectory_joint_max_positions = self._float_list_parameter(
            "pregrasp_trajectory_joint_max_positions", [3.14, 1.57, 1.38, 2.04], 4)
        for i in range(4):
            if self.pregrasp_trajectory_joint_min_positions[i] > self.pregrasp_trajectory_joint_max_positions[i]:
                self.pregrasp_trajectory_joint_min_positions[i], self.pregrasp_trajectory_joint_max_positions[i] = (
                    self.pregrasp_trajectory_joint_max_positions[i],
                    self.pregrasp_trajectory_joint_min_positions[i],
                )
        self.pre_grasp_arm_positions = self._real_pregrasp_target_positions(
            self.stay_arm_positions)
        self.grasp_arm_positions = list(self.pre_grasp_arm_positions)
        self.pre_place_arm_positions = self._level_gripper_pose(-math.pi, 0.82, -0.58)
        self.place_arm_positions = self._level_gripper_pose(-math.pi, 1.56, -0.47)
        self.base_approach_distance_m = float(
            self.get_parameter("base_approach_distance_m").value)
        self.base_transport_distance_m = float(
            self.get_parameter("base_transport_distance_m").value)
        self.base_turn_angle_rad = float(
            self.get_parameter("base_turn_angle_rad").value)
        self.base_turn_speed_radps = float(
            self.get_parameter("base_turn_speed_radps").value)
        self.post_place_move_distance_m = float(
            self.get_parameter("post_place_move_distance_m").value)
        self.post_place_move_speed_mps = float(
            self.get_parameter("post_place_move_speed_mps").value)
        self.post_place_reverse_distance_m = float(
            self.get_parameter("post_place_reverse_distance_m").value)
        self.post_place_reverse_speed_mps = float(
            self.get_parameter("post_place_reverse_speed_mps").value)
        self.post_place_turn_angle_rad = float(
            self.get_parameter("post_place_turn_angle_rad").value)
        self.post_place_turn_speed_radps = float(
            self.get_parameter("post_place_turn_speed_radps").value)
        self.base_x = 0.0
        self.base_y = 0.0
        self.base_yaw = 0.0
        self.place_base_pose = self._planned_base_pose_after_transport()
        self.pick_object_xyz = self._planned_object_odom_xyz(
            self.grasp_arm_positions,
            base_pose=(self.base_approach_distance_m, 0.0, 0.0),
        )
        self.released_object_xyz = None
        self.status_text = "READY"
        self.cargo_id = ""
        self.cargo_sequence = int(self.get_parameter("cargo_sequence_start").value)
        self.wheel_left = 0.0
        self.wheel_right = 0.0
        self.arm_positions = list(self.stay_arm_positions)
        self.gripper_position = self._object_gripper_open_position()
        self.active_trajectory = None
        self.last_gazebo_pose_update = 0.0
        self.warned_gazebo_pose_unavailable = False
        self.logged_gazebo_pose_ready = False
        self.gazebo_pose_future = None
        self.attached = False
        self.placed = False

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.bbox_pub = self.create_publisher(
            Float32MultiArray, self.get_parameter("bbox_topic").value, 10)
        self.eef_bbox_pub = self.create_publisher(
            Float32MultiArray, self.get_parameter("eef_bbox_topic").value, 10)
        self.joint_state_pub = self.create_publisher(
            JointState, self.get_parameter("joint_state_topic").value, 10)
        self.cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self.traj_pub = self.create_publisher(
            JointTrajectory, self.get_parameter("trajectory_topic").value, 10)
        marker_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, self.get_parameter("marker_topic").value, marker_qos)
        self.status_pub = self.create_publisher(
            String, self.get_parameter("status_topic").value, 10)
        current_id_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.cargo_event_pub = self.create_publisher(
            String, self.get_parameter("cargo_event_topic").value, 10)
        self.cargo_current_id_pub = self.create_publisher(
            String, self.get_parameter("cargo_current_id_topic").value, current_id_qos)
        self.gripper = ActionClient(
            self, GripperCommand, self.get_parameter("gripper_action_name").value)
        self.gazebo_pose_client = None
        if bool(self.get_parameter("sync_gazebo_object").value) and SetEntityPose is not None:
            self.gazebo_pose_client = self.create_client(
                SetEntityPose, str(self.get_parameter("gazebo_set_pose_service").value))
        elif bool(self.get_parameter("sync_gazebo_object").value):
            self.get_logger().warn(
                "ros_gz_interfaces is not available; Gazebo object pose sync disabled")

    def _level_gripper_pose(self, joint1, joint2, joint3):
        return [float(joint1), float(joint2), float(joint3), -float(joint2) - float(joint3)]

    def _float_list_parameter(self, name, default, expected_len):
        values = [float(v) for v in self.get_parameter(name).value]
        if len(values) != expected_len:
            self.get_logger().warn(
                f"{name} must have {expected_len} values; using default {default}")
            return [float(v) for v in default]
        return values

    def _clamp(self, value, lower, upper):
        return max(float(lower), min(float(value), float(upper)))

    def _transform_pregrasp_waypoint(self, raw_positions, current_positions, progress):
        transformed = [float(v) for v in raw_positions]
        if len(transformed) != 4:
            return transformed

        current = [float(v) for v in current_positions] if len(current_positions) == 4 else None
        progress = self._clamp(progress, 0.0, 1.0)

        if self.pregrasp_trajectory_transform_enabled:
            for i in range(4):
                if current is not None:
                    transformed[i] = (
                        current[i] +
                        (transformed[i] - current[i]) *
                        self.pregrasp_trajectory_joint_scales[i]
                    )
                else:
                    transformed[i] *= self.pregrasp_trajectory_joint_scales[i]
                transformed[i] += self.pregrasp_trajectory_joint_offsets[i] * progress

            if self.pregrasp_trajectory_use_pitch_blend:
                start_pitch = self.pregrasp_trajectory_target_pitch_rad
                if current is not None:
                    start_pitch = current[1] + current[2] + current[3]
                desired_pitch = (
                    start_pitch +
                    progress * (self.pregrasp_trajectory_target_pitch_rad - start_pitch)
                )
                pitch_based_joint4 = desired_pitch - transformed[1] - transformed[2]
                weight = self.pregrasp_trajectory_pitch_blend_weight
                transformed[3] = (1.0 - weight) * transformed[3] + weight * pitch_based_joint4

        for i in range(4):
            transformed[i] = self._clamp(
                transformed[i],
                self.pregrasp_trajectory_joint_min_positions[i],
                self.pregrasp_trajectory_joint_max_positions[i],
            )
        return transformed

    def _build_real_pregrasp_trajectory(self, current_positions=None):
        current = list(current_positions or self.arm_positions)
        if len(current) != 4:
            current = list(self.stay_arm_positions)

        points = []
        trajectory_time_s = 0.0

        if self.triangulation_extend_current_start_duration_s > 0.0:
            trajectory_time_s += self.triangulation_extend_current_start_duration_s
            points.append((
                self._transform_pregrasp_waypoint(current, current, 0.0),
                trajectory_time_s,
            ))

        trajectory_time_s += self.triangulation_extend_joint_duration_s
        points.append((
            self._transform_pregrasp_waypoint(
                self.triangulation_extend_joint_positions,
                current,
                1.0,
            ),
            trajectory_time_s,
        ))
        return points

    def _real_pregrasp_target_positions(self, current_positions=None):
        return self._build_real_pregrasp_trajectory(current_positions)[-1][0]

    def _real_pregrasp_wait_s(self, points):
        if not points:
            return self.triangulation_extend_joint_settle_s
        return max(float(t) for _, t in points) + self.triangulation_extend_joint_settle_s

    def run(self):
        self._wait_for_gazebo_pose_service()
        self._publish_ready_markers()
        self._sleep(float(self.get_parameter("start_delay_s").value))
        self._assign_cargo_id()
        self._status("DETECTED: object marker generated after planned base approach")
        self._log_gripper_width_targets()
        self._publish_cargo_event("assigned")
        self._publish_markers(attached=False, placed=False)
        self._sleep(1.5)

        self._status("BASE_APPROACH: driving robot before grasp")
        if not self._drive_base(
            self.base_approach_distance_m,
            float(self.get_parameter("base_approach_speed_mps").value),
            "base approach",
        ):
            return
        self._status("BASE_ALIGNED: robot reached grasping distance; publishing bbox")
        self._publish_bbox(repeats=10)

        self._status("APPROACH: moving arm to pre-grasp pose")
        if not self._send_gripper(self._object_gripper_open_position()):
            return
        pregrasp_points = self._build_real_pregrasp_trajectory(self.arm_positions)
        if not self._send_trajectory(pregrasp_points):
            return
        self._sleep(self._real_pregrasp_wait_s(pregrasp_points))
        self._status("FULL_REACH: arm fully extended at target")
        self._publish_eef_bbox(repeats=10)
        self._sleep(2.0)

        grasp_error_m, grasp_accuracy_percent = self._grasp_accuracy()
        self._status(
            "PICK: closing gripper and attaching object marker; "
            f"grasp_error_m={grasp_error_m:.4f}; "
            f"grasp_accuracy_percent={grasp_accuracy_percent:.2f}"
        )
        if not self._send_gripper(self._object_gripper_grasp_position()):
            return
        self._publish_cargo_event("picked")
        self._publish_markers(attached=True, placed=False)
        self._sleep(1.5)

        if self.direct_place_on_follower:
            self._status("HANDOFF_ALIGN: follower cargo deck ready behind leader")
            self._sleep(self.follower_handoff_wait_s)
            self._status("HANDOFF_LIFT: lifting object clear for direct follower placement")
            lift_points = self._build_real_pregrasp_trajectory(self.arm_positions)
            if not self._send_trajectory(lift_points):
                return
            self._sleep(self._real_pregrasp_wait_s(lift_points))
        else:
            self._status("CARRY: lifting object into transport pose")
            lift_points = self._build_real_pregrasp_trajectory(self.arm_positions)
            if not self._send_trajectory(lift_points):
                return
            self._sleep(self._real_pregrasp_wait_s(lift_points))

            if abs(self.base_turn_angle_rad) > 0.001:
                self._status("TURN_WITH_CARGO: rotating robot before transport")
                if not self._turn_base(
                    self.base_turn_angle_rad,
                    self.base_turn_speed_radps,
                    "cargo turn",
                ):
                    return
                self._sleep(0.5)

            if abs(self.base_transport_distance_m) > 0.001:
                self._status("MOVING_WITH_CARGO: driving turned heading to place location")
                if not self._drive_base(
                    self.base_transport_distance_m,
                    float(self.get_parameter("base_transport_speed_mps").value),
                    "cargo transport",
                ):
                    return
            self._status("ARRIVED_WITH_CARGO: robot reached place location")
            self._sleep(0.8)
            if self.place_on_follower:
                self._status("HANDOFF_ALIGN: follower closing cargo deck under arm")
                self._sleep(self.follower_handoff_wait_s)

        place_target = "follower cargo deck" if self.place_on_follower else "place target"
        place_action = "directly lowering" if self.direct_place_on_follower else "rotating arm 180 degrees and lowering"
        self._status(f"PLACE: {place_action} onto {place_target}")
        if not self._send_trajectory([
            (self.pre_place_arm_positions, 2.8),
            (self.place_arm_positions, 4.8),
        ]):
            return
        self._sleep(5.0)
        self._status("PLACE_REACH: arm fully extended behind robot")
        self._sleep(2.0)

        self._status("RELEASE: opening gripper at reached pose")
        if self.place_on_follower:
            self.released_object_xyz = None
        else:
            self.released_object_xyz = (
                self._carried_object_odom_xyz()
                or self._planned_object_odom_xyz(self.place_arm_positions)
            )
        if not self._send_gripper(self._object_gripper_open_position()):
            return
        self._publish_cargo_event("placed")
        self._publish_markers(attached=False, placed=True)
        self._sleep(1.2)

        if bool(self.get_parameter("return_to_stow").value):
            self._status("STAY: returning arm to saved navigation pose")
            if not self._send_trajectory([
                (self.pre_place_arm_positions, 1.6),
                (self.stay_arm_positions, 4.0),
            ]):
                return
            self._sleep(4.2)
            self._status("CARGO_LOADED: object placed; arm in stay pose")
        else:
            self._status("CARGO_LOADED: object placed; holding fully extended pose")

        if not self._run_post_place_departure():
            return

        self._status(
            "DONE: object placed and platoon completed reverse-turn-forward departure")

    def _run_post_place_departure(self):
        moved = False
        reverse_distance = abs(self.post_place_reverse_distance_m)
        if reverse_distance > 0.001:
            self._status("POST_PLACE_MOVE: backing up after follower cargo handoff")
            if not self._drive_base(
                -reverse_distance,
                self.post_place_reverse_speed_mps,
                "post-place reverse",
            ):
                return False
            moved = True
            self._sleep(0.4)

        if abs(self.post_place_turn_angle_rad) > 0.001:
            self._status("POST_PLACE_MOVE: changing direction after reverse")
            if not self._turn_base(
                self.post_place_turn_angle_rad,
                self.post_place_turn_speed_radps,
                "post-place turn",
            ):
                return False
            moved = True
            self._sleep(0.4)

        if abs(self.post_place_move_distance_m) > 0.001:
            self._status("POST_PLACE_MOVE: driving forward after direction change")
            if not self._drive_base(
                self.post_place_move_distance_m,
                self.post_place_move_speed_mps,
                "post-place forward",
            ):
                return False
            moved = True

        if moved:
            self._status(
                "POST_PLACE_ARRIVED: leader completed reverse-turn-forward departure")
            self._sleep(0.8)
        return True

    def _publish_ready_markers(self, repeats=3):
        msg = String()
        msg.data = self.status_text
        self.status_pub.publish(msg)
        for _ in range(max(1, repeats)):
            self._publish_markers(attached=False, placed=False)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

    def _sleep(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            self._publish_demo_state()
            self._publish_markers()
            rclpy.spin_once(self, timeout_sec=0.1)

    def _status(self, text):
        self.status_text = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def _assign_cargo_id(self):
        prefix = str(self.get_parameter("cargo_id_prefix").value)
        self.cargo_id = "{}-{:06d}".format(prefix, self.cargo_sequence)
        self.cargo_sequence += 1
        msg = String()
        msg.data = self.cargo_id
        self.cargo_current_id_pub.publish(msg)

    def _publish_cargo_event(self, event):
        if not self.cargo_id:
            self._assign_cargo_id()
        msg = String()
        stamp = self.get_clock().now().to_msg()
        msg.data = (
            '{{"cargo_id":"{}","event":"{}","stamp":{{"sec":{},"nanosec":{}}}}}'
        ).format(self.cargo_id, event, stamp.sec, stamp.nanosec)
        self.cargo_event_pub.publish(msg)
        current = String()
        current.data = self.cargo_id
        self.cargo_current_id_pub.publish(current)

    def _publish_bbox(self, repeats=1):
        msg = Float32MultiArray()
        msg.data = self.bbox
        for _ in range(max(1, repeats)):
            self.bbox_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)

    def _publish_eef_bbox(self, repeats=1):
        msg = Float32MultiArray()
        msg.data = self.eef_bbox
        for _ in range(max(1, repeats)):
            self.eef_bbox_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)

    def _wait_for_cmd_vel_subscriber(self, stage_name):
        require_subscriber = bool(
            self.get_parameter("require_cmd_vel_subscriber").value)

        wait_until = time.monotonic() + float(
            self.get_parameter("cmd_vel_wait_timeout_s").value)
        while (
            self.cmd_vel_pub.get_subscription_count() == 0
            and rclpy.ok()
            and time.monotonic() < wait_until
        ):
            self._publish_demo_state()
            self._publish_markers()
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.cmd_vel_pub.get_subscription_count() == 0:
            if require_subscriber:
                topic = str(self.get_parameter("cmd_vel_topic").value)
                self._status(
                    "ERROR: {} cmd_vel subscriber unavailable on {}".format(
                        stage_name, topic))
                return False
            self.get_logger().warn(
                "no /cmd_vel subscriber; publishing RViz demo TF for {}".format(
                    stage_name))
        return True

    def _drive_base(self, distance_m, speed_mps, stage_name):
        speed = abs(speed_mps)
        if speed < 0.01:
            speed = 0.10
        direction = 1.0 if distance_m >= 0.0 else -1.0
        duration_s = abs(distance_m) / speed
        if not self._wait_for_cmd_vel_subscriber(stage_name):
            return False

        end = time.monotonic() + duration_s
        last = time.monotonic()
        while rclpy.ok() and time.monotonic() < end:
            now = time.monotonic()
            dt = now - last
            last = now
            distance_step = direction * speed * dt
            self.base_x += distance_step * math.cos(self.base_yaw)
            self.base_y += distance_step * math.sin(self.base_yaw)
            self.wheel_left += direction * speed * dt / 0.033
            self.wheel_right += direction * speed * dt / 0.033
            msg = Twist()
            msg.linear.x = direction * speed
            self.cmd_vel_pub.publish(msg)
            self._publish_demo_state()
            self._publish_markers()
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
        self._stop_base()
        return True

    def _turn_base(self, angle_rad, speed_radps, stage_name):
        speed = abs(speed_radps)
        if speed < 0.05:
            speed = 0.30
        direction = 1.0 if angle_rad >= 0.0 else -1.0
        duration_s = abs(angle_rad) / speed
        if not self._wait_for_cmd_vel_subscriber(stage_name):
            return False

        wheel_separation_m = 0.287
        wheel_radius_m = 0.033
        end = time.monotonic() + duration_s
        last = time.monotonic()
        while rclpy.ok() and time.monotonic() < end:
            now = time.monotonic()
            dt = now - last
            last = now
            yaw_step = direction * speed * dt
            self.base_yaw = self._normalize_angle(self.base_yaw + yaw_step)
            wheel_step = yaw_step * 0.5 * wheel_separation_m / wheel_radius_m
            self.wheel_left -= wheel_step
            self.wheel_right += wheel_step
            msg = Twist()
            msg.angular.z = direction * speed
            self.cmd_vel_pub.publish(msg)
            self._publish_demo_state()
            self._publish_markers()
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
        self._stop_base()
        return True

    def _stop_base(self):
        stop = Twist()
        for _ in range(10):
            self.cmd_vel_pub.publish(stop)
            self._publish_demo_state()
            self._publish_markers()
            rclpy.spin_once(self, timeout_sec=0.03)
            time.sleep(0.03)

    def _publish_demo_state(self):
        self._update_demo_trajectory()
        self._publish_base_tf()
        self._publish_demo_joint_states()

    def _publish_base_tf(self):
        if not self.publish_demo_base_tf:
            return
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.base_x
        transform.transform.translation.y = self.base_y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = math.sin(0.5 * self.base_yaw)
        transform.transform.rotation.w = math.cos(0.5 * self.base_yaw)
        self.tf_broadcaster.sendTransform(transform)

    def _publish_demo_joint_states(self):
        if not self.publish_demo_joint_states:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.name = [
            "wheel_right_joint",
            "joint2",
            "wheel_left_joint",
            "joint1",
            "joint4",
            "gripper_left_joint",
            "gripper_right_joint",
            "joint3",
        ]
        msg.position = [
            self.wheel_right,
            self.arm_positions[1],
            self.wheel_left,
            self.arm_positions[0],
            self.arm_positions[3],
            self.gripper_position,
            self.gripper_position,
            self.arm_positions[2],
        ]
        self.joint_state_pub.publish(msg)

    def _update_demo_trajectory(self):
        if self.active_trajectory is None:
            return
        elapsed = time.monotonic() - self.active_trajectory["start_time"]
        prev_t = 0.0
        prev_positions = self.active_trajectory["start_positions"]
        for positions, target_t in self.active_trajectory["points"]:
            if elapsed <= target_t:
                span = max(target_t - prev_t, 0.001)
                ratio = min(max((elapsed - prev_t) / span, 0.0), 1.0)
                self.arm_positions = [
                    start + (end - start) * ratio
                    for start, end in zip(prev_positions, positions)
                ]
                return
            prev_t = target_t
            prev_positions = positions
        self.arm_positions = list(self.active_trajectory["points"][-1][0])
        self.active_trajectory = None

    def _start_demo_trajectory(self, points):
        self.active_trajectory = {
            "start_time": time.monotonic(),
            "start_positions": list(self.arm_positions),
            "points": [(list(positions), float(t)) for positions, t in points],
        }

    def _send_trajectory(self, points):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ["joint1", "joint2", "joint3", "joint4"]
        for positions, t in points:
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in positions]
            point.time_from_start = duration(float(t))
            msg.points.append(point)

        wait_until = time.monotonic() + float(
            self.get_parameter("trajectory_wait_timeout_s").value)
        while (
            self.traj_pub.get_subscription_count() == 0
            and rclpy.ok()
            and time.monotonic() < wait_until
        ):
            self._publish_demo_state()
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.traj_pub.get_subscription_count() > 0:
            self.traj_pub.publish(msg)
        else:
            if bool(self.get_parameter("require_trajectory_subscriber").value):
                topic = str(self.get_parameter("trajectory_topic").value)
                self._status(
                    "ERROR: arm trajectory subscriber unavailable on {}".format(topic))
                return False
            self.get_logger().warn(
                "arm trajectory subscriber is not available; using RViz demo joint states")
        self._start_demo_trajectory(points)
        return True

    def _send_gripper(self, position):
        position = max(
            self.gripper_joint_lower_m,
            min(float(position), self.gripper_joint_upper_m),
        )
        self.gripper_position = position
        timeout = float(self.get_parameter("gripper_wait_timeout_s").value)
        if not self.gripper.wait_for_server(timeout_sec=timeout):
            if bool(self.get_parameter("require_gripper_action_server").value):
                action = str(self.get_parameter("gripper_action_name").value)
                self._status(
                    "ERROR: gripper action server unavailable on {}".format(action))
                return False
            self.get_logger().warn("gripper action server is not available")
            return True

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = -1.0
        self.gripper.send_goal_async(goal)
        return True

    def _object_width_m(self):
        return max(0.0, float(self.object_size_xyz[1]))

    def _gripper_position_for_gap(self, gap_m):
        joint_position = (
            0.5 * max(0.0, float(gap_m)) - self.gripper_finger_home_half_gap_m)
        return max(
            self.gripper_joint_lower_m,
            min(joint_position, self.gripper_joint_upper_m),
        )

    def _object_gripper_open_position(self):
        return self._gripper_position_for_gap(
            self._object_width_m() + self.gripper_pre_grasp_clearance_m)

    def _object_gripper_grasp_position(self):
        return self._gripper_position_for_gap(
            max(0.0, self._object_width_m() - self.gripper_grasp_compression_m))

    def _log_gripper_width_targets(self):
        self.get_logger().info(
            "object width {:.3f} m -> gripper open {:.4f} m, grasp {:.4f} m".format(
                self._object_width_m(),
                self._object_gripper_open_position(),
                self._object_gripper_grasp_position(),
            )
        )

    def _grasp_accuracy(self):
        grasp_xyz = self._carried_object_odom_xyz()
        target_xyz = self.pick_object_xyz
        error_m = math.sqrt(
            sum((grasp_xyz[idx] - target_xyz[idx]) ** 2 for idx in range(3))
        )
        accuracy = 100.0 * max(
            0.0,
            1.0 - error_m / self.grasp_accuracy_tolerance_m,
        )
        return error_m, min(100.0, accuracy)

    def _publish_markers(self, attached=None, placed=None):
        self._publish_demo_state()
        if attached is not None:
            self.attached = attached
        if placed is not None:
            self.placed = placed
        attached = getattr(self, "attached", False)
        placed = getattr(self, "placed", False)

        markers = MarkerArray()
        markers.markers.append(self._object_marker(attached, placed))
        markers.markers.append(self._place_marker(placed))
        markers.markers.append(self._text_marker(attached, placed))
        markers.markers.append(self._cargo_id_marker(attached, placed))
        self.marker_pub.publish(markers)
        self._sync_gazebo_object_pose(attached, placed)

    def _object_marker(self, attached, placed):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pick_place_demo"
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = self.object_size_xyz[0]
        marker.scale.y = self.object_size_xyz[1]
        marker.scale.z = self.object_size_xyz[2]
        marker.color.r = 0.85
        marker.color.g = 0.04
        marker.color.b = 0.03
        marker.color.a = 0.95

        if attached:
            marker.header.frame_id = self.odom_frame
            xyz = self._carried_object_odom_xyz()
            marker.pose.position.x = xyz[0]
            marker.pose.position.y = xyz[1]
            marker.pose.position.z = xyz[2]
        elif placed and self.place_on_follower:
            marker.header.frame_id = self.follower_place_frame
            marker.frame_locked = True
            marker.pose.position.x = self.follower_place_xyz[0]
            marker.pose.position.y = self.follower_place_xyz[1]
            marker.pose.position.z = self.follower_place_xyz[2]
        else:
            marker.header.frame_id = self.place_frame if placed else self.object_frame
            xyz = self._placed_object_odom_xyz() if placed else self.pick_object_xyz
            marker.pose.position.x = xyz[0]
            marker.pose.position.y = xyz[1]
            marker.pose.position.z = xyz[2]
        marker.pose.orientation.w = 1.0
        return marker

    def _place_marker(self, placed):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.follower_place_frame if self.place_on_follower else self.place_frame
        marker.ns = "pick_place_demo"
        marker.id = 2
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        xyz = self.follower_place_xyz if self.place_on_follower else (
            self._placed_object_odom_xyz() if placed else self._planned_place_object_odom_xyz())
        marker.pose.position.x = xyz[0]
        marker.pose.position.y = xyz[1]
        marker.pose.position.z = xyz[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.01
        marker.color.r = 0.0
        marker.color.g = 0.65
        marker.color.b = 0.25
        marker.color.a = 0.35
        return marker

    def _text_marker(self, attached, placed):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "base_link"
        marker.ns = "pick_place_demo"
        marker.id = 3
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.25
        marker.pose.position.y = -0.26
        marker.pose.position.z = 0.42
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = self.status_text
        return marker

    def _cargo_id_marker(self, attached, placed):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pick_place_demo"
        marker.id = 4
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        if attached:
            marker.header.frame_id = self.odom_frame
            xyz = self._carried_object_odom_xyz()
            marker.pose.position.x = xyz[0]
            marker.pose.position.y = xyz[1]
            marker.pose.position.z = xyz[2] + 0.10
        elif placed and self.place_on_follower:
            marker.header.frame_id = self.follower_place_frame
            marker.frame_locked = True
            marker.pose.position.x = self.follower_place_xyz[0]
            marker.pose.position.y = self.follower_place_xyz[1]
            marker.pose.position.z = self.follower_place_xyz[2] + self.object_size_xyz[2] + 0.04
        else:
            marker.header.frame_id = self.place_frame if placed else self.object_frame
            xyz = self._placed_object_odom_xyz() if placed else self.pick_object_xyz
            marker.pose.position.x = xyz[0]
            marker.pose.position.y = xyz[1]
            marker.pose.position.z = xyz[2] + self.object_size_xyz[2] + 0.04
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.text = self.cargo_id if self.cargo_id else "UNASSIGNED"
        return marker

    def _sync_gazebo_object_pose(self, attached, placed):
        if self.gazebo_pose_client is None:
            return
        now = time.monotonic()
        update_period = float(self.get_parameter("gazebo_pose_update_period_s").value)
        if now - self.last_gazebo_pose_update < max(update_period, 0.02):
            return
        self.last_gazebo_pose_update = now
        if self.gazebo_pose_future is not None and not self.gazebo_pose_future.done():
            return

        pose = self._gazebo_object_pose(attached, placed)
        if pose is None:
            return
        if not self.gazebo_pose_client.service_is_ready():
            if not self.warned_gazebo_pose_unavailable:
                service_name = str(self.get_parameter("gazebo_set_pose_service").value)
                self.get_logger().warn(
                    "Gazebo set_pose service {} is not ready; RViz marker will still show the grasp".format(
                        service_name))
                self.warned_gazebo_pose_unavailable = True
            return
        if not self.logged_gazebo_pose_ready:
            service_name = str(self.get_parameter("gazebo_set_pose_service").value)
            object_name = str(self.get_parameter("gazebo_object_entity_name").value)
            self.get_logger().info(
                "syncing Gazebo object {} through {}".format(object_name, service_name))
            self.logged_gazebo_pose_ready = True

        request = SetEntityPose.Request()
        request.entity.name = str(self.get_parameter("gazebo_object_entity_name").value)
        request.entity.type = Entity.MODEL
        request.pose = pose
        self.gazebo_pose_future = self.gazebo_pose_client.call_async(request)

    def _wait_for_gazebo_pose_service(self):
        if self.gazebo_pose_client is None:
            return
        timeout_s = max(
            0.0, float(self.get_parameter("gazebo_pose_wait_timeout_s").value))
        service_name = str(self.get_parameter("gazebo_set_pose_service").value)
        if timeout_s <= 0.0:
            return

        deadline = time.monotonic() + timeout_s
        while (
            rclpy.ok()
            and not self.gazebo_pose_client.service_is_ready()
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.gazebo_pose_client.service_is_ready():
            self.get_logger().info(
                "Gazebo object pose service {} is ready".format(service_name))
        else:
            self.get_logger().warn(
                "Gazebo object pose service {} was not ready after {:.1f}s".format(
                    service_name, timeout_s))

    def _gazebo_object_pose(self, attached, placed):
        if attached:
            xyz = self._carried_object_odom_xyz()
        elif placed:
            xyz = self._placed_object_odom_xyz()
        else:
            xyz = self.pick_object_xyz

        pose = Pose()
        pose.position.x = xyz[0] + self.gazebo_world_origin_xyz[0]
        pose.position.y = xyz[1] + self.gazebo_world_origin_xyz[1]
        pose.position.z = xyz[2] + self.gazebo_world_origin_xyz[2]
        pose.orientation.w = 1.0
        return pose

    def _carried_object_odom_xyz(self):
        return self._planned_object_odom_xyz(self.arm_positions)

    def _attached_object_odom_xyz(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame, "end_effector_link", Time())
        except TransformException:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        offset = self._rotate_vector(rotation, self.attached_object_offset_xyz)
        return [
            translation.x + offset[0],
            translation.y + offset[1],
            translation.z + offset[2],
        ]

    def _placed_object_odom_xyz(self):
        if self.place_on_follower:
            follower_xyz = self._follower_place_object_odom_xyz()
            if follower_xyz is not None:
                return follower_xyz
        if self.released_object_xyz is not None:
            return self.released_object_xyz
        return self._planned_place_object_odom_xyz()

    def _planned_place_object_odom_xyz(self):
        if self.place_on_follower:
            follower_xyz = self._follower_place_object_odom_xyz()
            if follower_xyz is not None:
                return follower_xyz
        return self._planned_object_odom_xyz(
            self.place_arm_positions, base_pose=self.place_base_pose)

    def _follower_place_object_odom_xyz(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame, self.follower_place_frame, Time())
        except TransformException:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        offset = self._rotate_vector(rotation, self.follower_place_xyz)
        return [
            translation.x + offset[0],
            translation.y + offset[1],
            translation.z + offset[2],
        ]

    def _planned_object_odom_xyz(self, arm_positions, base_pose=None, base_x=None):
        matrix = self._planned_end_effector_matrix(arm_positions)
        translation = [matrix[0][3], matrix[1][3], matrix[2][3]]
        offset = self._rotate_matrix_vector(matrix, self.attached_object_offset_xyz)
        if base_pose is None:
            if base_x is not None:
                base_pose = (float(base_x), 0.0, 0.0)
            else:
                base_pose = (self.base_x, self.base_y, self.base_yaw)
        base_x, base_y, base_yaw = base_pose
        local_x = translation[0] + offset[0]
        local_y = translation[1] + offset[1]
        rotated_x, rotated_y = self._rotate_xy(local_x, local_y, base_yaw)
        return [
            base_x + rotated_x,
            base_y + rotated_y,
            translation[2] + offset[2],
        ]

    def _planned_base_pose_after_transport(self):
        yaw = self._normalize_angle(self.base_turn_angle_rad)
        return (
            self.base_approach_distance_m
            + self.base_transport_distance_m * math.cos(yaw),
            self.base_transport_distance_m * math.sin(yaw),
            yaw,
        )

    def _planned_end_effector_matrix(self, arm_positions):
        joint1, joint2, joint3, joint4 = arm_positions
        matrix = self._identity_matrix()
        for transform in [
            self._translation_matrix(-0.092, 0.0, 0.178),
            self._translation_matrix(0.012, 0.0, 0.017),
            self._rotation_z_matrix(joint1),
            self._translation_matrix(0.0, 0.0, 0.0595),
            self._rotation_y_matrix(joint2),
            self._translation_matrix(0.024, 0.0, 0.128),
            self._rotation_y_matrix(joint3),
            self._translation_matrix(0.124, 0.0, 0.0),
            self._rotation_y_matrix(joint4),
            self._translation_matrix(0.126, 0.0, 0.0),
        ]:
            matrix = self._matrix_multiply(matrix, transform)
        return matrix

    def _identity_matrix(self):
        return [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def _translation_matrix(self, x, y, z):
        matrix = self._identity_matrix()
        matrix[0][3] = x
        matrix[1][3] = y
        matrix[2][3] = z
        return matrix

    def _rotation_y_matrix(self, angle):
        c = math.cos(angle)
        s = math.sin(angle)
        return [
            [c, 0.0, s, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [-s, 0.0, c, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def _rotation_z_matrix(self, angle):
        c = math.cos(angle)
        s = math.sin(angle)
        return [
            [c, -s, 0.0, 0.0],
            [s, c, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def _matrix_multiply(self, lhs, rhs):
        return [
            [
                sum(lhs[row][idx] * rhs[idx][col] for idx in range(4))
                for col in range(4)
            ]
            for row in range(4)
        ]

    def _rotate_matrix_vector(self, matrix, vector):
        return [
            sum(matrix[row][idx] * vector[idx] for idx in range(3))
            for row in range(3)
        ]

    def _rotate_xy(self, x, y, yaw):
        c = math.cos(yaw)
        s = math.sin(yaw)
        return [c * x - s * y, s * x + c * y]

    def _normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _rotate_vector(self, quaternion, vector):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        vx, vy, vz = vector

        tx = 2.0 * (y * vz - z * vy)
        ty = 2.0 * (z * vx - x * vz)
        tz = 2.0 * (x * vy - y * vx)

        return [
            vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx),
        ]


def main():
    rclpy.init()
    node = SimPickPlaceDemo()
    try:
        node.run()
        while rclpy.ok():
            node._publish_markers()
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        try:
            node.destroy_node()
        except RCLError:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
