#!/usr/bin/env python3
import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JsonToJointStates(Node):
    def __init__(self, path: Path, hz: float, max_age_sec: float):
        super().__init__("json_to_jointstates_50hz")
        self.path = path
        self.max_age_sec = float(max_age_sec)
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(1.0 / float(hz), self.tick)
        self.last_warn = 0.0
        self.get_logger().info(f"Publishing /joint_states from {path} at {hz} Hz")

    def tick(self):
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
        except Exception as e:
            now = time.time()
            if now - self.last_warn > 2.0:
                self.get_logger().warn(f"Cannot read {self.path}: {e}")
                self.last_warn = now
            return

        if not data.get("ok"):
            return

        age = time.time() - float(data.get("saved_unix_sec", 0.0))
        if age > self.max_age_sec:
            now = time.time()
            if now - self.last_warn > 2.0:
                self.get_logger().warn(f"Joint JSON stale: age={age:.3f}s")
                self.last_warn = now
            return

        names = list(data["joint_names"])
        positions = [float(x) for x in data["position"]]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        self.pub.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--path", default="/tmp/g1_joint_positions.json")
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--max-age-sec", type=float, default=0.5)
    args = ap.parse_args()

    rclpy.init()
    node = JsonToJointStates(Path(args.path), args.hz, args.max_age_sec)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
