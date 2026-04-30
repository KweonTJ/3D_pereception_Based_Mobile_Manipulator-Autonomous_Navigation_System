#!/usr/bin/env python3
"""RViz-focused simulated pick-and-place demonstration.

This node is intentionally a visualization/demo sequencer. It publishes the
same bbox used by the tracker, commands the Gazebo arm controller through a
joint trajectory, and publishes RViz markers so the object is visible while it
is picked and placed.
"""

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
        self.declare_parameter("eef_bbox_topic", "/target/eef_init_bbox")
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
        self.declare_parameter("start_delay_s", 4.0)
        self.declare_parameter("object_frame", "odom")
        self.declare_parameter("place_frame", "odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_demo_base_tf", True)
        self.declare_parameter("publish_demo_joint_states", True)
        self.declare_parameter("return_to_stow", False)
        self.declare_parameter("object_xyz", [1.30, 0.0, 0.115])
        self.declare_parameter("place_xyz", [0.72, 0.0, 0.115])
        self.declare_parameter("approach_distance_m", 1.00)
        self.declare_parameter("approach_speed_mps", 0.12)
        self.declare_parameter("cmd_vel_wait_timeout_s", 20.0)
        self.declare_parameter("sync_gazebo_object", True)
        self.declare_parameter("gazebo_set_pose_service", "/world/default/set_pose")
        self.declare_parameter("gazebo_object_entity_name", "grasp_test_cube")
        self.declare_parameter("gazebo_world_origin_xyz", [-2.0, -0.5, 0.0])
        self.declare_parameter("gazebo_pose_update_period_s", 0.10)

        self.bbox = [float(v) for v in self.get_parameter("bbox").value]
        self.eef_bbox = [float(v) for v in self.get_parameter("eef_bbox").value]
        self.object_frame = str(self.get_parameter("object_frame").value)
        self.place_frame = str(self.get_parameter("place_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_demo_base_tf = bool(self.get_parameter("publish_demo_base_tf").value)
        self.publish_demo_joint_states = bool(
            self.get_parameter("publish_demo_joint_states").value)
        self.object_xyz = [float(v) for v in self.get_parameter("object_xyz").value]
        self.place_xyz = [float(v) for v in self.get_parameter("place_xyz").value]
        self.gazebo_world_origin_xyz = [
            float(v) for v in self.get_parameter("gazebo_world_origin_xyz").value]
        self.status_text = "READY"
        self.cargo_id = ""
        self.cargo_sequence = int(self.get_parameter("cargo_sequence_start").value)
        self.base_x = 0.0
        self.wheel_left = 0.0
        self.wheel_right = 0.0
        self.arm_positions = [0.0, 0.10, 0.02, -0.80]
        self.gripper_position = 0.019
        self.active_trajectory = None
        self.last_gazebo_pose_update = 0.0
        self.warned_gazebo_pose_unavailable = False

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
        self.marker_pub = self.create_publisher(
            MarkerArray, self.get_parameter("marker_topic").value, 10)
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

    def run(self):
        self._sleep(float(self.get_parameter("start_delay_s").value))
        self._assign_cargo_id()
        self._status("DETECTED: far object marker published in odom")
        self._publish_cargo_event("assigned")
        self._publish_markers(attached=False, placed=False)
        self._sleep(1.5)

        self._status("BASE_APPROACH: driving robot close to object")
        self._drive_base(
            float(self.get_parameter("approach_distance_m").value),
            float(self.get_parameter("approach_speed_mps").value),
        )
        self._status("BASE_ALIGNED: object is within manipulator reach; publishing bbox")
        self._publish_bbox(repeats=10)

        self._status("APPROACH: moving arm to pre-grasp pose")
        self._send_gripper(0.019)
        self._send_trajectory([
            ([0.0, 0.10, 0.02, -0.80], 1.5),
            ([0.0, 0.82, -0.58, -0.35], 3.5),
            ([0.0, 1.32, -0.94, -0.23], 5.5),
        ])
        self._sleep(5.8)
        self._status("FULL_REACH: arm fully extended at target")
        self._publish_eef_bbox(repeats=10)
        self._sleep(2.0)

        self._status("PICK: closing gripper and attaching object marker")
        self._send_gripper(-0.015)
        self._publish_cargo_event("picked")
        self._publish_markers(attached=True, placed=False)
        self._sleep(1.5)

        self._status("PLACE: lifting and moving to rear place pose")
        self._send_trajectory([
            ([0.0, 0.82, -0.58, -0.35], 1.6),
            ([-3.141592653589793, 0.82, -0.58, -0.35], 4.2),
            ([-3.141592653589793, 1.32, -0.94, -0.23], 6.2),
        ])
        self._sleep(6.4)
        self._status("PLACE_REACH: arm fully extended behind robot")
        self._sleep(2.0)

        self._status("RELEASE: opening gripper at place target")
        self._send_gripper(0.019)
        self._publish_cargo_event("placed")
        self._publish_markers(attached=False, placed=True)
        self._sleep(1.2)

        self._status("DONE: object placed; holding fully extended pose")
        if bool(self.get_parameter("return_to_stow").value):
            self._send_trajectory([
                ([-3.141592653589793, 0.82, -0.58, -0.35], 1.6),
                ([0.0, 0.10, 0.02, -0.80], 4.0),
            ])

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

    def _drive_base(self, distance_m, speed_mps):
        speed = abs(speed_mps)
        if speed < 0.01:
            speed = 0.10
        direction = 1.0 if distance_m >= 0.0 else -1.0
        duration_s = abs(distance_m) / speed

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
            self.get_logger().warn(
                "no /cmd_vel subscriber; publishing RViz demo TF for base approach")

        end = time.monotonic() + duration_s
        last = time.monotonic()
        while rclpy.ok() and time.monotonic() < end:
            now = time.monotonic()
            dt = now - last
            last = now
            self.base_x += direction * speed * dt
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

    def _stop_base(self):
        stop = Twist()
        for _ in range(10):
            self.cmd_vel_pub.publish(stop)
            self._publish_demo_state()
            self._publish_markers()
            rclpy.spin_once(self, timeout_sec=0.03)
            time.sleep(0.03)

    def _publish_demo_state(self):
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
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

    def _publish_demo_joint_states(self):
        if not self.publish_demo_joint_states:
            return
        self._update_demo_trajectory()
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

        wait_until = time.monotonic() + 2.0
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
            self.get_logger().warn(
                "arm trajectory subscriber is not available; using RViz demo joint states")
        self._start_demo_trajectory(points)

    def _send_gripper(self, position):
        self.gripper_position = float(position)
        if not self.gripper.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("gripper action server is not available")
            return

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = -1.0
        self.gripper.send_goal_async(goal)

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
        markers.markers.append(self._place_marker())
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
        marker.scale.x = 0.06
        marker.scale.y = 0.06
        marker.scale.z = 0.08
        marker.color.r = 0.85
        marker.color.g = 0.04
        marker.color.b = 0.03
        marker.color.a = 0.95

        if attached:
            marker.header.frame_id = "end_effector_link"
            marker.frame_locked = True
            marker.pose.position.x = 0.08
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
        else:
            marker.header.frame_id = self.place_frame if placed else self.object_frame
            xyz = self.place_xyz if placed else self.object_xyz
            marker.pose.position.x = xyz[0]
            marker.pose.position.y = xyz[1]
            marker.pose.position.z = xyz[2]
        marker.pose.orientation.w = 1.0
        return marker

    def _place_marker(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.place_frame
        marker.ns = "pick_place_demo"
        marker.id = 2
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = self.place_xyz[0]
        marker.pose.position.y = self.place_xyz[1]
        marker.pose.position.z = self.place_xyz[2]
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
            marker.header.frame_id = "end_effector_link"
            marker.frame_locked = True
            marker.pose.position.x = 0.08
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.10
        else:
            marker.header.frame_id = self.place_frame if placed else self.object_frame
            xyz = self.place_xyz if placed else self.object_xyz
            marker.pose.position.x = xyz[0]
            marker.pose.position.y = xyz[1]
            marker.pose.position.z = xyz[2] + 0.12
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

        pose = self._gazebo_object_pose(attached, placed)
        if pose is None:
            return
        if not self.gazebo_pose_client.service_is_ready():
            if not self.warned_gazebo_pose_unavailable:
                self.get_logger().warn(
                    "Gazebo set_pose service is not ready; RViz marker will still show the grasp")
                self.warned_gazebo_pose_unavailable = True
            return

        request = SetEntityPose.Request()
        request.entity.name = str(self.get_parameter("gazebo_object_entity_name").value)
        request.entity.type = Entity.MODEL
        request.pose = pose
        self.gazebo_pose_client.call_async(request)

    def _gazebo_object_pose(self, attached, placed):
        if attached:
            xyz = self._attached_object_odom_xyz()
            if xyz is None:
                return None
        elif placed:
            xyz = self.place_xyz
        else:
            xyz = self.object_xyz

        pose = Pose()
        pose.position.x = xyz[0] + self.gazebo_world_origin_xyz[0]
        pose.position.y = xyz[1] + self.gazebo_world_origin_xyz[1]
        pose.position.z = xyz[2] + self.gazebo_world_origin_xyz[2]
        pose.orientation.w = 1.0
        return pose

    def _attached_object_odom_xyz(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame, "end_effector_link", Time())
        except TransformException:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        offset = self._rotate_vector(rotation, [0.08, 0.0, 0.0])
        return [
            translation.x + offset[0],
            translation.y + offset[1],
            translation.z + offset[2],
        ]

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
