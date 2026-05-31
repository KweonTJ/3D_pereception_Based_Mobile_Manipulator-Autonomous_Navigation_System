#!/usr/bin/env python3

from copy import deepcopy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class JointTrajectoryTransformer(Node):
    """Mirror selected joint trajectory deltas around the current joint state."""

    def __init__(self):
        super().__init__("joint_trajectory_transformer")
        self.input_topic = self.declare_parameter(
            "input_topic", "/arm_controller/joint_trajectory_raw").value
        self.output_topic = self.declare_parameter(
            "output_topic", "/arm_controller/joint_trajectory").value
        self.joint_state_topic = self.declare_parameter(
            "joint_state_topic", "/joint_states").value
        self.reverse_joint_names = set(
            self.declare_parameter("reverse_joint_names", ["joint3"]).value)
        self.preserve_roll_sum = bool(
            self.declare_parameter("preserve_roll_sum", True).value)
        self.roll_parent_joint = self.declare_parameter(
            "roll_parent_joint", "joint2").value
        self.roll_reversed_joint = self.declare_parameter(
            "roll_reversed_joint", "joint3").value
        self.roll_compensation_joint = self.declare_parameter(
            "roll_compensation_joint", "joint4").value

        if self.input_topic == self.output_topic:
            raise RuntimeError("input_topic and output_topic must be different")

        self.latest_joint_positions = {}
        self.warned_missing_reference = False
        self.logged_first_transform = False

        self.publisher = self.create_publisher(JointTrajectory, self.output_topic, 10)
        self.create_subscription(
            JointState, self.joint_state_topic, self.on_joint_state, 10)
        self.create_subscription(
            JointTrajectory, self.input_topic, self.on_trajectory, 10)

        self.get_logger().info(
            "joint trajectory transformer: %s -> %s, reverse delta joints=%s, "
            "preserve_roll_sum=%s"
            % (
                self.input_topic,
                self.output_topic,
                sorted(self.reverse_joint_names),
                self.preserve_roll_sum,
            )
        )

    def on_joint_state(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.latest_joint_positions[name] = position

    def on_trajectory(self, msg):
        transformed = deepcopy(msg)
        references = self._reference_positions(msg)
        if references is None:
            if not self.warned_missing_reference:
                self.get_logger().warn(
                    "dropping trajectory until joint3 reference is available "
                    "from /joint_states"
                )
                self.warned_missing_reference = True
            return

        reverse_indices = [
            index for index, name in enumerate(msg.joint_names)
            if name in self.reverse_joint_names
        ]
        roll_indices = self._roll_indices(msg) if self.preserve_roll_sum else None
        for point in transformed.points:
            original_positions = list(point.positions)
            original_velocities = list(point.velocities)
            original_accelerations = list(point.accelerations)
            for index in reverse_indices:
                joint_name = msg.joint_names[index]
                reference = references[joint_name]
                if len(point.positions) > index:
                    original_value = point.positions[index]
                    mirrored_value = 2.0 * reference - original_value
                    point.positions[index] = mirrored_value
                    self._preserve_roll_value(
                        point.positions,
                        original_positions,
                        index,
                        mirrored_value - original_value,
                        roll_indices)
                if len(point.velocities) > index:
                    original_value = point.velocities[index]
                    point.velocities[index] = -point.velocities[index]
                    self._preserve_roll_value(
                        point.velocities,
                        original_velocities,
                        index,
                        point.velocities[index] - original_value,
                        roll_indices)
                if len(point.accelerations) > index:
                    original_value = point.accelerations[index]
                    point.accelerations[index] = -point.accelerations[index]
                    self._preserve_roll_value(
                        point.accelerations,
                        original_accelerations,
                        index,
                        point.accelerations[index] - original_value,
                        roll_indices)
                if len(point.effort) > index:
                    point.effort[index] = -point.effort[index]

        if reverse_indices and not self.logged_first_transform:
            self.get_logger().info(
                "mirroring joint trajectory deltas around reference positions: %s"
                % {
                    name: round(references[name], 4)
                    for name in sorted(references.keys())
                }
            )
            self.logged_first_transform = True

        self.publisher.publish(transformed)

    def _reference_positions(self, msg):
        references = {}
        for joint_name in msg.joint_names:
            if joint_name not in self.reverse_joint_names:
                continue
            references[joint_name] = self.latest_joint_positions.get(joint_name)

        if any(position is None for position in references.values()):
            return None
        return references

    def _roll_indices(self, msg):
        try:
            return {
                "parent": msg.joint_names.index(self.roll_parent_joint),
                "reversed": msg.joint_names.index(self.roll_reversed_joint),
                "compensation": msg.joint_names.index(self.roll_compensation_joint),
            }
        except ValueError:
            return None

    def _preserve_roll_value(
        self,
        values,
        original_values,
        reversed_index,
        reversed_delta,
        roll_indices,
    ):
        if roll_indices is None:
            return
        if reversed_index != roll_indices["reversed"]:
            return
        compensation_index = roll_indices["compensation"]
        if len(values) <= compensation_index or len(original_values) <= compensation_index:
            return
        values[compensation_index] = original_values[compensation_index] - reversed_delta


def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryTransformer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
