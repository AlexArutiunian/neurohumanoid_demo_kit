#!/usr/bin/env python3
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import JointState
from unitree_hg.msg import LowState


joint_names = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]


class LowStateSampler(Node):
    def __init__(self, hz: float, timeout_sec: float):
        super().__init__("lowstate_sample_to_jointstates")
        self.hz = float(hz)
        self.timeout_sec = float(timeout_sec)

        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.get_logger().info(f"Sampling /lowstate -> /joint_states at {self.hz:.2f} Hz")

    def publish_once(self):
        ok, low = wait_for_message(
            LowState,
            self,
            "/lowstate",
            qos_profile=self.qos,
            time_to_wait=self.timeout_sec,
        )

        if not ok or low is None:
            self.get_logger().warn("No /lowstate sample received")
            return False

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = [float(low.motor_state[i].q) for i in range(len(joint_names))]
        self.pub.publish(msg)
        return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hz", type=float, default=2.0)
    ap.add_argument("--timeout-sec", type=float, default=0.2)
    args = ap.parse_args()

    rclpy.init()
    node = LowStateSampler(args.hz, args.timeout_sec)

    period = 1.0 / float(args.hz)

    try:
        while rclpy.ok():
            t0 = time.time()
            node.publish_once()
            dt = time.time() - t0
            time.sleep(max(0.0, period - dt))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
