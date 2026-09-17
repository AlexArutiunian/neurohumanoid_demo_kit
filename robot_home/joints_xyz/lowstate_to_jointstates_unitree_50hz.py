#!/usr/bin/env python3
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_


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

latest_positions = None
lock = threading.Lock()


def lowstate_cb(msg: LowState_):
    global latest_positions
    with lock:
        latest_positions = [float(msg.motor_state[i].q) for i in range(len(joint_names))]


class JointStateBridge(Node):
    def __init__(self):
        super().__init__("lowstate_to_jointstates_unitree_50hz")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(0.02, self.publish_joint_states)
        self.get_logger().info("Unitree rt/lowstate -> /joint_states at 50 Hz")

    def publish_joint_states(self):
        global latest_positions
        with lock:
            if latest_positions is None:
                return
            positions = list(latest_positions)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = positions
        self.pub.publish(msg)


def main():
    # ВАЖНО: сначала инициализируем ROS2/rclpy,
    # потом Unitree DDS. Иначе иногда ломается RMW handle.
    rclpy.init()

    ChannelFactoryInitialize(0)

    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(lowstate_cb, 10)

    node = JointStateBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
