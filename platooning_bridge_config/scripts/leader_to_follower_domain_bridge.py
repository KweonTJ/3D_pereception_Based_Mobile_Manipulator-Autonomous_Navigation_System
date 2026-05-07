#!/usr/bin/env python3

import importlib
import sys
import threading
import time

import rclpy
import yaml
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions


def message_type(type_name):
    package, name = type_name.split("/msg/", 1)
    module = importlib.import_module(f"{package}.msg")
    return getattr(module, name)


def qos_profile(qos_config):
    qos_config = qos_config or {}
    profile = QoSProfile(depth=int(qos_config.get("depth", 10)))

    reliability = str(qos_config.get("reliability", "reliable")).lower()
    if reliability == "best_effort":
        profile.reliability = ReliabilityPolicy.BEST_EFFORT
    else:
        profile.reliability = ReliabilityPolicy.RELIABLE

    durability = str(qos_config.get("durability", "volatile")).lower()
    if durability == "transient_local":
        profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
    else:
        profile.durability = DurabilityPolicy.VOLATILE

    return profile


class BridgeEndpoint:
    def __init__(self, config_path):
        with open(config_path, "r", encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file)

        self.name = str(config.get("name", "leader_to_follower_bridge"))
        self.from_domain = int(config["from_domain"])
        self.to_domain = int(config["to_domain"])
        self.topics = config.get("topics", {})
        if not self.topics:
            raise RuntimeError(f"no topics configured in {config_path}")

        self.from_context = Context()
        self.to_context = Context()
        rclpy.init(
            context=self.from_context,
            domain_id=self.from_domain,
            signal_handler_options=SignalHandlerOptions.NO,
        )
        rclpy.init(
            context=self.to_context,
            domain_id=self.to_domain,
            signal_handler_options=SignalHandlerOptions.NO,
        )

        self.from_node = Node(f"{self.name}_from_d{self.from_domain}", context=self.from_context)
        self.to_node = Node(f"{self.name}_to_d{self.to_domain}", context=self.to_context)
        self.publishers = []
        self.subscriptions = []
        self._create_topic_bridges()

        self.from_executor = SingleThreadedExecutor(context=self.from_context)
        self.to_executor = SingleThreadedExecutor(context=self.to_context)
        self.from_executor.add_node(self.from_node)
        self.to_executor.add_node(self.to_node)

    def _create_topic_bridges(self):
        for topic, topic_config in self.topics.items():
            msg_type = message_type(str(topic_config["type"]))
            qos = qos_profile(topic_config.get("qos"))
            publisher = self.to_node.create_publisher(msg_type, topic, qos)

            def relay(msg, pub=publisher):
                pub.publish(msg)

            subscription = self.from_node.create_subscription(msg_type, topic, relay, qos)
            self.publishers.append(publisher)
            self.subscriptions.append(subscription)
            self.from_node.get_logger().info(
                f"bridging {topic} [{topic_config['type']}] "
                f"domain {self.from_domain} -> {self.to_domain}"
            )

    def spin(self):
        threads = [
            threading.Thread(target=self.from_executor.spin, daemon=True),
            threading.Thread(target=self.to_executor.spin, daemon=True),
        ]
        for thread in threads:
            thread.start()

        self.from_node.get_logger().info(
            f"{self.name} started: domain {self.from_domain} -> {self.to_domain}"
        )
        try:
            while self.from_context.ok() and self.to_context.ok():
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def shutdown(self):
        self.from_executor.shutdown()
        self.to_executor.shutdown()
        self.from_node.destroy_node()
        self.to_node.destroy_node()
        if self.from_context.ok():
            self.from_context.shutdown()
        if self.to_context.ok():
            self.to_context.shutdown()


def main():
    if len(sys.argv) != 2:
        print("usage: leader_to_follower_domain_bridge.py <bridge_config.yaml>", file=sys.stderr)
        return 2

    bridge = BridgeEndpoint(sys.argv[1])
    bridge.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
