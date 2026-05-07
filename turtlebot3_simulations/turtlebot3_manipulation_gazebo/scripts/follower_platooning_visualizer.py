#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformException
from tf2_ros import TransformListener


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    half_yaw = 0.5 * yaw
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


class FollowerPlatooningVisualizer(Node):

    def __init__(self):
        super().__init__("follower_platooning_visualizer")

        self.declare_parameter("leader_odom_topic", "/odom")
        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("follower_frame", "follower_base_footprint")
        self.declare_parameter("leader_reference_frame", "imu_link")
        self.declare_parameter("follower_reference_frame", "follower_imu_link")
        self.declare_parameter("follower_odom_topic", "/follower/odom")
        self.declare_parameter("follower_joint_state_topic", "/follower/joint_states")
        self.declare_parameter(
            "follower_wheel_joints",
            ["follower_wheel_left_joint", "follower_wheel_right_joint"],
        )
        self.declare_parameter("target_distance_m", 0.45)
        self.declare_parameter("handoff_distance_m", 0.45)
        self.declare_parameter("initial_offset_x_m", -0.45)
        self.declare_parameter("initial_offset_y_m", 0.0)
        self.declare_parameter("initial_yaw_offset_rad", 0.0)
        self.declare_parameter("wheel_radius_m", 0.033)
        self.declare_parameter("wheel_separation_m", 0.288)
        self.declare_parameter("status_topic", "/mp_control/pick_place_status")
        self.declare_parameter("max_linear_speed_mps", 0.24)
        self.declare_parameter("max_angular_speed_radps", 1.2)
        self.declare_parameter("linear_gain", 0.85)
        self.declare_parameter("angular_gain", 2.4)
        self.declare_parameter("distance_deadband_m", 0.03)
        self.declare_parameter("publish_rate_hz", 30.0)

        self.leader_odom_topic = self._string_param("leader_odom_topic")
        self.parent_frame = self._string_param("parent_frame")
        self.follower_frame = self._string_param("follower_frame")
        self.leader_reference_frame = self._string_param("leader_reference_frame")
        self.follower_reference_frame = self._string_param("follower_reference_frame")
        self.follower_odom_topic = self._string_param("follower_odom_topic")
        self.follower_joint_state_topic = self._string_param("follower_joint_state_topic")
        self.follower_wheel_joints = [
            str(name) for name in self.get_parameter("follower_wheel_joints").value
        ]
        self.target_distance_m = self._float_param("target_distance_m")
        self.handoff_distance_m = self._float_param("handoff_distance_m")
        self.initial_offset_x_m = self._float_param("initial_offset_x_m")
        self.initial_offset_y_m = self._float_param("initial_offset_y_m")
        self.initial_yaw_offset_rad = self._float_param("initial_yaw_offset_rad")
        self.wheel_radius_m = max(0.001, self._float_param("wheel_radius_m"))
        self.wheel_separation_m = max(0.0, self._float_param("wheel_separation_m"))
        self.status_topic = self._string_param("status_topic")
        self.max_linear_speed_mps = self._float_param("max_linear_speed_mps")
        self.max_angular_speed_radps = self._float_param("max_angular_speed_radps")
        self.linear_gain = self._float_param("linear_gain")
        self.angular_gain = self._float_param("angular_gain")
        self.distance_deadband_m = self._float_param("distance_deadband_m")
        publish_rate_hz = max(1.0, self._float_param("publish_rate_hz"))

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_pub = self.create_publisher(Odometry, self.follower_odom_topic, 10)
        self.joint_state_pub = self.create_publisher(
            JointState, self.follower_joint_state_topic, 10
        )
        self.create_subscription(Odometry, self.leader_odom_topic, self._leader_odom_cb, 10)
        self.create_subscription(String, self.status_topic, self._status_cb, 10)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._timer_cb)

        self.leader_pose = None
        self.follower_x = 0.0
        self.follower_y = 0.0
        self.follower_yaw = 0.0
        self.last_time = None
        self.last_log_time = 0.0
        self.initialized = False
        self.last_linear_speed = 0.0
        self.last_angular_speed = 0.0
        self.left_wheel_position = 0.0
        self.right_wheel_position = 0.0
        self.handoff_active = False

        self.get_logger().info(
            "Follower platooning visualizer started: "
            f"target_distance={self.target_distance_m:.2f} m, "
            f"handoff_distance={self.handoff_distance_m:.2f} m, "
            f"reference={self.leader_reference_frame}->{self.follower_reference_frame}, "
            f"leader_odom={self.leader_odom_topic}"
        )

    def _string_param(self, name):
        return str(self.get_parameter(name).value)

    def _float_param(self, name):
        return float(self.get_parameter(name).value)

    def _leader_odom_cb(self, msg):
        pose = msg.pose.pose
        self.leader_pose = (
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(pose.orientation),
        )

        if not self.initialized:
            leader_x, leader_y, leader_yaw = self.leader_pose
            cos_yaw = math.cos(leader_yaw)
            sin_yaw = math.sin(leader_yaw)
            self.follower_x = (
                leader_x
                + self.initial_offset_x_m * cos_yaw
                - self.initial_offset_y_m * sin_yaw
            )
            self.follower_y = (
                leader_y
                + self.initial_offset_x_m * sin_yaw
                + self.initial_offset_y_m * cos_yaw
            )
            self.follower_yaw = normalize_angle(leader_yaw + self.initial_yaw_offset_rad)
            self.initialized = True
            self.get_logger().info(
                "Follower initialized behind leader at "
                f"({self.follower_x:.2f}, {self.follower_y:.2f})"
            )

    def _status_cb(self, msg):
        status = msg.data.upper()
        stage = status.split(":", 1)[0].strip()
        should_handoff = stage in {
            "HANDOFF_ALIGN",
            "PLACE",
            "PLACE_REACH",
            "RELEASE",
            "DONE",
        }
        should_reset = stage in {
            "DETECTED",
            "BASE_APPROACH",
            "MOVING_WITH_CARGO",
            "TURN_WITH_CARGO",
        }
        if should_handoff and not self.handoff_active:
            self.handoff_active = True
            self.get_logger().info(
                f"HANDOFF: closing to {self.handoff_distance_m:.2f} m for cargo transfer"
            )
        elif should_reset and self.handoff_active:
            self.handoff_active = False
            self.get_logger().info(
                f"FOLLOW: returning to {self.target_distance_m:.2f} m spacing"
            )

    def _timer_cb(self):
        if self.leader_pose is None or not self.initialized:
            return

        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            self._publish_follower(now)
            return

        dt = (now - self.last_time).nanoseconds * 1.0e-9
        self.last_time = now
        if dt <= 0.0 or dt > 1.0:
            self._publish_follower(now)
            return

        leader_x, leader_y, leader_yaw = self._leader_reference_pose()
        active_distance = (
            self.handoff_distance_m if self.handoff_active else self.target_distance_m
        )
        target_x = leader_x - active_distance * math.cos(leader_yaw)
        target_y = leader_y - active_distance * math.sin(leader_yaw)
        follower_ref_x, follower_ref_y, _ = self._follower_reference_pose()

        dx = target_x - follower_ref_x
        dy = target_y - follower_ref_y
        target_error = math.hypot(dx, dy)

        if target_error > self.distance_deadband_m:
            desired_heading = math.atan2(dy, dx)
            heading_error = normalize_angle(desired_heading - self.follower_yaw)
            heading_scale = clamp(math.cos(heading_error), 0.0, 1.0)
            linear_speed = min(
                self.max_linear_speed_mps,
                self.linear_gain * target_error,
            ) * heading_scale
            angular_error = heading_error
        else:
            linear_speed = 0.0
            angular_error = normalize_angle(leader_yaw - self.follower_yaw)

        angular_speed = clamp(
            self.angular_gain * angular_error,
            -self.max_angular_speed_radps,
            self.max_angular_speed_radps,
        )

        self.follower_x += linear_speed * math.cos(self.follower_yaw) * dt
        self.follower_y += linear_speed * math.sin(self.follower_yaw) * dt
        self.follower_yaw = normalize_angle(self.follower_yaw + angular_speed * dt)
        self.last_linear_speed = linear_speed
        self.last_angular_speed = angular_speed
        self._integrate_wheels(linear_speed, angular_speed, dt)

        self._publish_follower(now)
        self._log_spacing(now, leader_x, leader_y, active_distance)

    def _lookup_pose(self, frame_id):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.parent_frame,
                frame_id,
                Time(),
            )
        except TransformException:
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            translation.x,
            translation.y,
            yaw_from_quaternion(rotation),
        )

    def _leader_reference_pose(self):
        pose = self._lookup_pose(self.leader_reference_frame)
        if pose is not None:
            return pose
        return self.leader_pose

    def _follower_reference_pose(self):
        pose = self._lookup_pose(self.follower_reference_frame)
        if pose is not None:
            return pose
        return (self.follower_x, self.follower_y, self.follower_yaw)

    def _integrate_wheels(self, linear_speed, angular_speed, dt):
        half_track = 0.5 * self.wheel_separation_m
        left_speed = linear_speed - angular_speed * half_track
        right_speed = linear_speed + angular_speed * half_track
        self.left_wheel_position += (left_speed / self.wheel_radius_m) * dt
        self.right_wheel_position += (right_speed / self.wheel_radius_m) * dt

    def _publish_follower(self, stamp):
        qx, qy, qz, qw = quaternion_from_yaw(self.follower_yaw)

        transform = TransformStamped()
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.follower_frame
        transform.transform.translation.x = self.follower_x
        transform.transform.translation.y = self.follower_y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

        odom = Odometry()
        odom.header = transform.header
        odom.child_frame_id = self.follower_frame
        odom.pose.pose.position.x = self.follower_x
        odom.pose.pose.position.y = self.follower_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = transform.transform.rotation
        odom.twist.twist.linear.x = self.last_linear_speed
        odom.twist.twist.angular.z = self.last_angular_speed
        self.odom_pub.publish(odom)

        joint_state = JointState()
        joint_state.header.stamp = stamp.to_msg()
        joint_state.name = self.follower_wheel_joints
        joint_positions = {
            "follower_wheel_left_joint": self.left_wheel_position,
            "follower_wheel_right_joint": self.right_wheel_position,
        }
        joint_state.position = [
            joint_positions.get(name, 0.0) for name in self.follower_wheel_joints
        ]
        self.joint_state_pub.publish(joint_state)

    def _log_spacing(self, now, leader_x, leader_y, active_distance):
        now_sec = now.nanoseconds * 1.0e-9
        if now_sec - self.last_log_time < 2.0:
            return
        self.last_log_time = now_sec
        follower_x, follower_y, _ = self._follower_reference_pose()
        spacing = math.hypot(leader_x - follower_x, leader_y - follower_y)
        self.get_logger().info(
            f"FOLLOWING: imu_spacing={spacing:.2f} m "
            f"target={active_distance:.2f} m "
            f"frames={self.leader_reference_frame}->{self.follower_reference_frame} "
            f"v={self.last_linear_speed:.2f} m/s "
            f"w={self.last_angular_speed:.2f} rad/s"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FollowerPlatooningVisualizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except RCLError:
            pass


if __name__ == "__main__":
    main()
