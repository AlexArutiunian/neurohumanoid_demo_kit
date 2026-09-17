#!/usr/bin/env python3
import argparse
import json
import os
import signal
import sys
import time
import threading
from datetime import datetime
from pathlib import Path
from typing import List, Optional

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

JOINT_NAMES: List[str] = [
    "LeftHipPitch", "LeftHipRoll", "LeftHipYaw", "LeftKnee", "LeftAnklePitch", "LeftAnkleRoll",
    "RightHipPitch", "RightHipRoll", "RightHipYaw", "RightKnee", "RightAnklePitch", "RightAnkleRoll",
    "WaistYaw", "WaistRoll", "WaistPitch",
    "LeftShoulderPitch", "LeftShoulderRoll", "LeftShoulderYaw", "LeftElbow", "LeftWristRoll", "LeftWristPitch", "LeftWristYaw",
    "RightShoulderPitch", "RightShoulderRoll", "RightShoulderYaw", "RightElbow", "RightWristRoll", "RightWristPitch", "RightWristYaw",
    "Joint29", "Joint30", "Joint31", "Joint32", "Joint33", "Joint34",
]

JOINT_NAME_MAPPING = {
    "LeftHipPitch": "left_hip_pitch_joint",
    "LeftHipRoll": "left_hip_roll_joint",
    "LeftHipYaw": "left_hip_yaw_joint",
    "LeftKnee": "left_knee_joint",
    "LeftAnklePitch": "left_ankle_pitch_joint",
    "LeftAnkleRoll": "left_ankle_roll_joint",
    "RightHipPitch": "right_hip_pitch_joint",
    "RightHipRoll": "right_hip_roll_joint",
    "RightHipYaw": "right_hip_yaw_joint",
    "RightKnee": "right_knee_joint",
    "RightAnklePitch": "right_ankle_pitch_joint",
    "RightAnkleRoll": "right_ankle_roll_joint",
    "WaistYaw": "waist_yaw_joint",
    "WaistRoll": "waist_roll_joint",
    "WaistPitch": "waist_pitch_joint",
    "LeftShoulderPitch": "left_shoulder_pitch_joint",
    "LeftShoulderRoll": "left_shoulder_roll_joint",
    "LeftShoulderYaw": "left_shoulder_yaw_joint",
    "LeftElbow": "left_elbow_joint",
    "LeftWristRoll": "left_wrist_roll_joint",
    "LeftWristPitch": "left_wrist_pitch_joint",
    "LeftWristYaw": "left_wrist_yaw_joint",
    "RightShoulderPitch": "right_shoulder_pitch_joint",
    "RightShoulderRoll": "right_shoulder_roll_joint",
    "RightShoulderYaw": "right_shoulder_yaw_joint",
    "RightElbow": "right_elbow_joint",
    "RightWristRoll": "right_wrist_roll_joint",
    "RightWristPitch": "right_wrist_pitch_joint",
    "RightWristYaw": "right_wrist_yaw_joint",
}

class JointStateRecorder:
    def __init__(self, topic: str, out_dir: str, run_name: str, rate_hz: float):
        self.topic = topic
        self.out_dir = Path(out_dir).resolve() / run_name
        self.out_dir.mkdir(parents=True, exist_ok=True)

        self.json_path = self.out_dir / "actual_joint_states.json"
        self.meta_path = self.out_dir / "recorder_meta.json"

        self.sub = ChannelSubscriber(topic, LowState_)
        self.frames = []
        self.lock = threading.Lock()

        self.start_time: Optional[float] = None
        self.last_frame_time: Optional[float] = None
        self.last_saved_sample_time = 0.0
        self.min_dt = 1.0 / max(float(rate_hz), 1.0)

        self.msg_count = 0
        self.running = True
        self.saved = False

        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def start(self):
        self.sub.Init(self.on_state, 50)
        print(f"[DDS REC] topic: {self.topic}")
        print(f"[DDS REC] out:   {self.json_path}")

    def close(self):
        try:
            self.sub.Close()
        except Exception:
            pass

    def on_state(self, msg: LowState_):
        if not self.running:
            return

        now = time.time()
        if now - self.last_saved_sample_time < self.min_dt:
            return
        self.last_saved_sample_time = now

        if self.start_time is None:
            self.start_time = now
            self.last_frame_time = now
            print("[DDS REC] first LowState received")

        duration = now - self.last_frame_time if self.last_frame_time else 0.0
        self.last_frame_time = now

        frame_data = []
        for i, motor in enumerate(msg.motor_state):
            if i >= len(JOINT_NAMES):
                continue
            raw_name = JOINT_NAMES[i]
            mapped_name = JOINT_NAME_MAPPING.get(raw_name, raw_name.lower())
            angle_deg = float(motor.q) * 57.2957795131

            frame_data.append({
                "name": mapped_name,
                "angle": angle_deg
            })

        frame_obj = {
            "frame": frame_data,
            "duration": round(duration, 10)
        }

        with self.lock:
            self.frames.append(frame_obj)
            self.msg_count += 1
            if self.msg_count % 100 == 0:
                print(f"[DDS REC] frames: {self.msg_count}")

    def signal_handler(self, sig, frame):
        self.running = False
        print(f"\n[DDS REC] stop signal {sig}, saving...")
        self.save_trajectory()
        self.close()
        sys.exit(0)

    def save_trajectory(self):
        with self.lock:
            if self.saved:
                return
            frames_copy = list(self.frames)
            self.saved = True

        with open(self.json_path, "w", encoding="utf-8") as f:
            json.dump(frames_copy, f, ensure_ascii=False, indent=2)

        meta = {
            "saved_at": datetime.now().isoformat(),
            "topic": self.topic,
            "frames": len(frames_copy),
            "total_duration": sum(x.get("duration", 0.0) for x in frames_copy),
        }
        with open(self.meta_path, "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)

        print(f"[DDS REC] saved: {self.json_path}")
        print(f"[DDS REC] frames: {len(frames_copy)}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("network_interface", nargs="?", default="eth0")
    parser.add_argument("--topic", default="rt/lowstate")

    # совместимость с joystick_launch_json_record.py
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--run-name", required=True)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--target-frame", default="")
    parser.add_argument("--links", default="")

    args = parser.parse_args()

    ChannelFactoryInitialize(0, args.network_interface)

    recorder = JointStateRecorder(
        topic=args.topic,
        out_dir=args.out_dir,
        run_name=args.run_name,
        rate_hz=args.rate_hz,
    )
    recorder.start()

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        recorder.signal_handler(signal.SIGINT, None)

if __name__ == "__main__":
    main()
