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
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


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
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("trajectory_topic", "/arm_controller/joint_trajectory")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("marker_topic", "/mp_control/pick_place_markers")
        self.declare_parameter("status_topic", "/mp_control/pick_place_status")
        self.declare_parameter("gripper_action_name", "/gripper_controller/gripper_cmd")
        self.declare_parameter("start_delay_s", 4.0)
        self.declare_parameter("object_frame", "odom")
        self.declare_parameter("place_frame", "odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_demo_base_tf", True)
        self.declare_parameter("publish_demo_joint_states", True)
        self.declare_parameter("object_xyz", [1.30, 0.0, 0.115])
        self.declare_parameter("place_xyz", [1.20, 0.20, 0.115])
        self.declare_parameter("approach_distance_m", 1.00)
        self.declare_parameter("approach_speed_mps", 0.12)

        self.bbox = [float(v) for v in self.get_parameter("bbox").value]
        self.object_frame = str(self.get_parameter("object_frame").value)
        self.place_frame = str(self.get_parameter("place_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_demo_base_tf = bool(self.get_parameter("publish_demo_base_tf").value)
        self.publish_demo_joint_states = bool(
            self.get_parameter("publish_demo_joint_states").value)
        self.object_xyz = [float(v) for v in self.get_parameter("object_xyz").value]
        self.place_xyz = [float(v) for v in self.get_parameter("place_xyz").value]
        self.status_text = "READY"
        self.base_x = 0.0
        self.wheel_left = 0.0
        self.wheel_right = 0.0
        self.arm_positions = [0.0, 0.10, 0.02, -0.80]
        self.gripper_position = 0.019
        self.active_trajectory = None

        self.tf_broadcaster = TransformBroadcaster(self)
        self.bbox_pub = self.create_publisher(
            Float32MultiArray, self.get_parameter("bbox_topic").value, 10)
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
        self.gripper = ActionClient(
            self, GripperCommand, self.get_parameter("gripper_action_name").value)

    def run(self):
        self._sleep(float(self.get_parameter("start_delay_s").value))
        self._status("DETECTED: far object marker published in odom")
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

        self._status("PICK: closing gripper and attaching object marker")
        self._send_gripper(-0.015)
        self._publish_markers(attached=True, placed=False)
        self._sleep(1.5)

        self._status("PLACE: lifting and moving to place pose")
        self._send_trajectory([
            ([0.0, 0.82, -0.58, -0.35], 1.6),
            ([0.78, 0.82, -0.58, -0.35], 3.2),
            ([0.78, 1.32, -0.94, -0.23], 5.2),
        ])
        self._sleep(5.4)

        self._status("RELEASE: opening gripper at place target")
        self._send_gripper(0.019)
        self._publish_markers(attached=False, placed=True)
        self._sleep(1.2)

        self._status("DONE: pick-and-place RViz demo complete")
        self._send_trajectory([
            ([0.78, 0.82, -0.58, -0.35], 1.6),
            ([0.0, 0.10, 0.02, -0.80], 3.2),
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

    def _publish_bbox(self, repeats=1):
        msg = Float32MultiArray()
        msg.data = self.bbox
        for _ in range(max(1, repeats)):
            self.bbox_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)

    def _drive_base(self, distance_m, speed_mps):
        speed = abs(speed_mps)
        if speed < 0.01:
            speed = 0.10
        direction = 1.0 if distance_m >= 0.0 else -1.0
        duration_s = abs(distance_m) / speed

        wait_until = time.monotonic() + 1.0
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
        self.marker_pub.publish(markers)

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
