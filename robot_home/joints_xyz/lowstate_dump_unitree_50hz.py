#!/usr/bin/env python3
import json
import os
import time
import threading
import argparse

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

lock = threading.Lock()
latest_positions = None
latest_recv = 0.0


def cb(msg: LowState_):
    global latest_positions, latest_recv
    with lock:
        latest_positions = [float(msg.motor_state[i].q) for i in range(len(joint_names))]
        latest_recv = time.time()


def atomic_write_json(path, obj):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False)
    os.replace(tmp, path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/g1_joint_positions.json")
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--iface", default=None)
    args = ap.parse_args()

    if args.iface:
        ChannelFactoryInitialize(0, args.iface)
    else:
        ChannelFactoryInitialize(0)

    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(cb, 10)

    print(f"[unitree_dump] writing {args.out} at {args.hz} Hz")

    dt = 1.0 / float(args.hz)
    while True:
        with lock:
            pos = None if latest_positions is None else list(latest_positions)
            recv = latest_recv

        if pos is not None:
            atomic_write_json(args.out, {
                "ok": True,
                "saved_unix_sec": time.time(),
                "lowstate_recv_unix_sec": recv,
                "joint_names": joint_names,
                "position": pos,
            })

        time.sleep(dt)


if __name__ == "__main__":
    main()
