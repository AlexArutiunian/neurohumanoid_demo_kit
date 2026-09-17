#!/usr/bin/env python3
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
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


class LowStateToJointStatesThrottled(Node):
    def __init__(self):
        super().__init__("lowstate_to_jointstates_throttled")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.lock = threading.Lock()
        self.latest_positions = None

        self.pub = self.create_publisher(JointState, "/joint_states", 10)

        self.sub = self.create_subscription(
            LowState,
            "/lowstate",
            self.cb,
            qos,
        )

        # 50 Hz вместо 400-500 Hz
        self.timer = self.create_timer(0.02, self.publish_joint_states)

        self.get_logger().info("Subscribed to /lowstate, publishing /joint_states at 50 Hz")

    def cb(self, msg):
        with self.lock:
            self.latest_positions = [msg.motor_state[i].q for i in range(len(joint_names))]

    def publish_joint_states(self):
        with self.lock:
            if self.latest_positions is None:
                return
            positions = list(self.latest_positions)

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = joint_names
        out.position = positions
        self.pub.publish(out)


def main():
    rclpy.init()
    node = LowStateToJointStatesThrottled()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
