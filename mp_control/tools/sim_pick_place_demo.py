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
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
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
        self.declare_parameter("trajectory_topic", "/arm_controller/joint_trajectory")
        self.declare_parameter("marker_topic", "/mp_control/pick_place_markers")
        self.declare_parameter("status_topic", "/mp_control/pick_place_status")
        self.declare_parameter("gripper_action_name", "/gripper_controller/gripper_cmd")
        self.declare_parameter("start_delay_s", 4.0)
        self.declare_parameter("object_xyz", [0.40, 0.0, 0.115])
        self.declare_parameter("place_xyz", [0.18, 0.28, 0.115])

        self.bbox = [float(v) for v in self.get_parameter("bbox").value]
        self.object_xyz = [float(v) for v in self.get_parameter("object_xyz").value]
        self.place_xyz = [float(v) for v in self.get_parameter("place_xyz").value]

        self.bbox_pub = self.create_publisher(
            Float32MultiArray, self.get_parameter("bbox_topic").value, 10)
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
        self._status("DETECTED: publishing target bbox and object marker")
        self._publish_bbox(repeats=10)
        self._publish_markers(attached=False, placed=False)

        self._status("APPROACH: moving arm to pre-grasp pose")
        self._send_gripper(0.019)
        self._send_trajectory([
            ([0.0, 0.10, 0.02, -0.80], 1.5),
            ([0.0, 0.42, -0.20, -1.05], 3.5),
            ([0.0, 0.66, -0.46, -0.86], 5.5),
        ])
        self._sleep(5.8)

        self._status("PICK: closing gripper and attaching object marker")
        self._send_gripper(-0.015)
        self._publish_markers(attached=True, placed=False)
        self._sleep(1.5)

        self._status("PLACE: lifting and moving to place pose")
        self._send_trajectory([
            ([0.0, 0.24, -0.20, -1.15], 1.6),
            ([0.78, 0.24, -0.20, -1.15], 3.2),
            ([0.78, 0.58, -0.42, -0.90], 5.2),
        ])
        self._sleep(5.4)

        self._status("RELEASE: opening gripper at place target")
        self._send_gripper(0.019)
        self._publish_markers(attached=False, placed=True)
        self._sleep(1.2)

        self._status("DONE: pick-and-place RViz demo complete")
        self._send_trajectory([
            ([0.78, 0.24, -0.20, -1.15], 1.6),
            ([0.0, 0.10, 0.02, -0.80], 3.2),
        ])

    def _sleep(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            self._publish_markers()
            rclpy.spin_once(self, timeout_sec=0.1)

    def _status(self, text):
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

    def _send_trajectory(self, points):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ["joint1", "joint2", "joint3", "joint4"]
        for positions, t in points:
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in positions]
            point.time_from_start = duration(float(t))
            msg.points.append(point)

        while self.traj_pub.get_subscription_count() == 0 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.traj_pub.publish(msg)

    def _send_gripper(self, position):
        if not self.gripper.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("gripper action server is not available")
            return

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = -1.0
        self.gripper.send_goal_async(goal)

    def _publish_markers(self, attached=None, placed=None):
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
            marker.header.frame_id = "base_link"
            xyz = self.place_xyz if placed else self.object_xyz
            marker.pose.position.x = xyz[0]
            marker.pose.position.y = xyz[1]
            marker.pose.position.z = xyz[2]
        marker.pose.orientation.w = 1.0
        return marker

    def _place_marker(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "base_link"
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
        if attached:
            marker.text = "PICK: object attached to gripper"
        elif placed:
            marker.text = "PLACE: object released"
        else:
            marker.text = "DETECTED: target bbox initialized"
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
