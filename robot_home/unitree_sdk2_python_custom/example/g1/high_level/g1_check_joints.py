#!/usr/bin/env python3
import argparse
import time
from typing import List, Optional, Tuple

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_

"""
python3 g1_check_joints.py
"""
JOINT_NAMES: List[str] = [
    "LeftHipPitch", "LeftHipRoll", "LeftHipYaw", "LeftKnee", "LeftAnklePitch", "LeftAnkleRoll",
    "RightHipPitch", "RightHipRoll", "RightHipYaw", "RightKnee", "RightAnklePitch", "RightAnkleRoll",
    "WaistYaw", "WaistRoll", "WaistPitch",
    "LeftShoulderPitch", "LeftShoulderRoll", "LeftShoulderYaw", "LeftElbow", "LeftWristRoll", "LeftWristPitch", "LeftWristYaw",
    "RightShoulderPitch", "RightShoulderRoll", "RightShoulderYaw", "RightElbow", "RightWristRoll", "RightWristPitch", "RightWristYaw",
    "Joint29", "Joint30", "Joint31", "Joint32", "Joint33", "Joint34",
]


class StiffnessMonitor:
    def __init__(self, topic: str, eps: float):
        self.topic = topic
        self.eps = eps
        self.sub = ChannelSubscriber(topic, LowCmd_)
        self.prev: Optional[List[Tuple[float, float]]] = None
        self.msg_count = 0

    def start(self):
        self.sub.Init(self.on_cmd, 50)

    def close(self):
        self.sub.Close()

    def on_cmd(self, msg: LowCmd_):
        self.msg_count += 1

        cur: List[Tuple[float, float]] = []
        for i in range(len(msg.motor_cmd)):
            kp = float(msg.motor_cmd[i].kp)
            kd = float(msg.motor_cmd[i].kd)
            cur.append((kp, kd))

        ts = time.strftime("%H:%M:%S")

        if self.prev is None:
            print(f"[{ts}] First lowcmd received. Current joint stiffness (kp, kd):")
            for i, (kp, kd) in enumerate(cur):
                name = JOINT_NAMES[i] if i < len(JOINT_NAMES) else f"Joint{i}"
                print(f"  {i:02d} {name:<18} kp={kp:7.3f}  kd={kd:7.3f}")
            self.prev = cur
            return

        changed = []
        for i, ((pkp, pkd), (kp, kd)) in enumerate(zip(self.prev, cur)):
            if abs(kp - pkp) >= self.eps or abs(kd - pkd) >= self.eps:
                changed.append((i, pkp, pkd, kp, kd))

        if not changed:
            return

        print(f"[{ts}] Stiffness changed on {len(changed)} joint(s):")
        for i, pkp, pkd, kp, kd in changed:
            name = JOINT_NAMES[i] if i < len(JOINT_NAMES) else f"Joint{i}"
            print(
                f"  {i:02d} {name:<18} "
                f"kp {pkp:7.3f} -> {kp:7.3f}, "
                f"kd {pkd:7.3f} -> {kd:7.3f}"
            )

        self.prev = cur


def main():
    parser = argparse.ArgumentParser(
        description="Monitor G1 joint stiffness (kp/kd) from LowCmd and print only when changed."
    )
    parser.add_argument("network_interface", nargs="?", default="eth0")
    parser.add_argument("--topic", default="rt/lowcmd")
    parser.add_argument("--eps", type=float, default=1e-3, help="Minimum change to report")
    parser.add_argument("--warn-no-msg", type=float, default=3.0, help="Warn interval if no messages")
    args = parser.parse_args()

    ChannelFactoryInitialize(0, args.network_interface)

    mon = StiffnessMonitor(topic=args.topic, eps=args.eps)
    mon.start()

    print(f"Interface: {args.network_interface}")
    print(f"Topic: {args.topic}")
    print(f"Change threshold: {args.eps}")
    print("Waiting for LowCmd... Press Ctrl+C to stop.")

    last_count = 0
    last_warn = time.monotonic()

    try:
        while True:
            time.sleep(0.2)
            now = time.monotonic()
            if mon.msg_count == last_count and (now - last_warn) >= args.warn_no_msg:
                print(
                    f"No messages on '{args.topic}' yet. "
                    "If needed, try --topic rt/lf/lowcmd"
                )
                last_warn = now
            else:
                last_count = mon.msg_count
    finally:
        mon.close()


if __name__ == "__main__":
    main()

