#!/usr/bin/env python3
import argparse
import json
import os
import select
import signal
import sys
import time
import threading
from datetime import datetime
from typing import Dict, List, Optional, Tuple

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_


RAD2DEG = 57.29577951308232


JOINT_NAMES: List[str] = [
    "LeftHipPitch", "LeftHipRoll", "LeftHipYaw", "LeftKnee", "LeftAnklePitch", "LeftAnkleRoll",
    "RightHipPitch", "RightHipRoll", "RightHipYaw", "RightKnee", "RightAnklePitch", "RightAnkleRoll",
    "WaistYaw", "WaistRoll", "WaistPitch",
    "LeftShoulderPitch", "LeftShoulderRoll", "LeftShoulderYaw", "LeftElbow",
    "LeftWristRoll", "LeftWristPitch", "LeftWristYaw",
    "RightShoulderPitch", "RightShoulderRoll", "RightShoulderYaw", "RightElbow",
    "RightWristRoll", "RightWristPitch", "RightWristYaw",
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


LEFT_ARM_BASE_JOINTS = [
    "LeftShoulderPitch",
    "LeftShoulderRoll",
    "LeftShoulderYaw",
    "LeftElbow",
    "LeftWristRoll",
]

RIGHT_ARM_BASE_JOINTS = [
    "RightShoulderPitch",
    "RightShoulderRoll",
    "RightShoulderYaw",
    "RightElbow",
    "RightWristRoll",
]

LEFT_WRIST_PY_JOINTS = [
    "LeftWristPitch",
    "LeftWristYaw",
]

RIGHT_WRIST_PY_JOINTS = [
    "RightWristPitch",
    "RightWristYaw",
]

WAIST_JOINTS = [
    "WaistYaw",
    "WaistRoll",
    "WaistPitch",
]

LEG_JOINTS = [
    "LeftHipPitch", "LeftHipRoll", "LeftHipYaw", "LeftKnee", "LeftAnklePitch", "LeftAnkleRoll",
    "RightHipPitch", "RightHipRoll", "RightHipYaw", "RightKnee", "RightAnklePitch", "RightAnkleRoll",
]


class KeyframeRecorder:
    def __init__(
        self,
        *,
        topic: str,
        output_dir: str,
        output_name: Optional[str],
        controlled_sdk_names: List[str],
        default_duration: float,
        angle_eps: float,
        round_digits: int,
        no_sparse: bool,
        min_changed_joints: int,
        stream_interval: float,
        stream_sparse: bool,
    ):
        self.topic = topic
        self.output_dir = output_dir
        self.output_name = output_name

        self.controlled_sdk_names = set(controlled_sdk_names)
        self.default_duration = float(default_duration)
        self.angle_eps = float(angle_eps)
        self.round_digits = int(round_digits)
        self.no_sparse = bool(no_sparse)
        self.min_changed_joints = int(min_changed_joints)

        self.stream_interval = float(stream_interval)
        self.stream_sparse = bool(stream_sparse)

        self.stream_active = False
        self.stream_until: Optional[float] = None
        self.last_stream_capture_time: Optional[float] = None
        self.stream_frame_count = 0

        self.sub = ChannelSubscriber(topic, LowState_)

        self.lock = threading.Lock()
        self.latest_pose: Optional[Dict[str, float]] = None
        self.latest_vel: Optional[Dict[str, float]] = None
        self.prev_saved_pose: Optional[Dict[str, float]] = None

        self.frames: List[Dict] = []
        self.msg_count = 0
        self.start_time: Optional[float] = None
        self.running = True
        self.saved_path: Optional[str] = None

        signal.signal(signal.SIGINT, self.signal_handler)

    def start(self):
        self.sub.Init(self.on_state, 50)
        print(f"[INFO] Recording from topic: {self.topic}")
        print(f"[INFO] Controlled SDK joints: {', '.join(sorted(self.controlled_sdk_names))}")
        print("[INFO] Commands:")
        print("       Enter             capture sparse keyframe")
        print("       enter 5           capture sparse keyframe after 5 seconds")
        print("       f                 capture full keyframe")
        print("       d 1.2             set default duration for manual keyframes")
        print("       r                 toggle stream recording every --stream-interval seconds")
        print("       record 5          stream-record for 5 seconds")
        print("       stop              stop stream recording")
        print("       s name.json       save current motion and clear buffer")
        print("       save name.json    save current motion and clear buffer")
        print("       status            print status")
        print("       q                 save remaining buffer and quit")
        print("       q name.json       save remaining buffer as name and quit")
        print("       Ctrl+C            save remaining buffer and quit")

    def close(self):
        try:
            self.sub.Close()
        except Exception:
            pass

    def on_state(self, msg: LowState_):
        if not self.running:
            return

        current_pose: Dict[str, float] = {}
        current_vel: Dict[str, float] = {}

        for i, motor in enumerate(msg.motor_state):
            if i >= len(JOINT_NAMES):
                continue

            sdk_name = JOINT_NAMES[i]

            if sdk_name not in self.controlled_sdk_names:
                continue

            mapped_name = JOINT_NAME_MAPPING.get(sdk_name)
            if mapped_name is None:
                continue

            q_deg = float(motor.q) * RAD2DEG
            dq_deg_s = float(motor.dq) * RAD2DEG

            current_pose[mapped_name] = q_deg
            current_vel[mapped_name] = dq_deg_s

        with self.lock:
            self.latest_pose = current_pose
            self.latest_vel = current_vel
            self.msg_count += 1
            if self.start_time is None:
                self.start_time = time.time()

    def _rounded(self, x: float) -> float:
        return round(float(x), self.round_digits)

    def _build_frame(
        self,
        current_pose: Dict[str, float],
        *,
        full: bool,
    ) -> Tuple[List[Dict], Dict[str, float]]:
        rounded_pose = {
            name: self._rounded(angle)
            for name, angle in current_pose.items()
        }

        if full or self.no_sparse or self.prev_saved_pose is None:
            frame = [
                {"name": name, "angle": rounded_pose[name]}
                for name in sorted(rounded_pose.keys())
            ]
            return frame, rounded_pose

        frame = []
        for name in sorted(rounded_pose.keys()):
            angle = rounded_pose[name]
            prev_angle = self.prev_saved_pose.get(name)

            if prev_angle is None or abs(angle - prev_angle) >= self.angle_eps:
                frame.append({"name": name, "angle": angle})

        return frame, rounded_pose

    def capture_keyframe(
        self,
        *,
        full: bool = False,
        duration: Optional[float] = None,
        source: str = "manual",
        allow_empty: bool = False,
    ) -> bool:
        with self.lock:
            if self.latest_pose is None:
                print("[WARN] No LowState received yet; keyframe not captured.")
                return False
            current_pose = dict(self.latest_pose)

        first = self.prev_saved_pose is None
        full = bool(full or first)

        frame, rounded_pose = self._build_frame(current_pose, full=full)

        if not frame and not allow_empty:
            if source == "manual":
                print("[INFO] Pose change below threshold; keyframe skipped.")
            return False

        if not full and len(frame) < self.min_changed_joints and not allow_empty:
            if source == "manual":
                print(
                    f"[INFO] Only {len(frame)} joints changed; "
                    f"min_changed_joints={self.min_changed_joints}; keyframe skipped."
                )
            return False

        dur = self.default_duration if duration is None else float(duration)
        dur = max(0.01, dur)

        obj = {
            "frame": frame,
            "duration": round(dur, 4),
        }

        self.frames.append(obj)
        self.prev_saved_pose = rounded_pose

        if source == "manual":
            mode = "FULL" if full else "SPARSE"
            print(
                f"[OK] Captured keyframe #{len(self.frames)} "
                f"({mode}, joints={len(frame)}, duration={dur:.3f}s)"
            )
        elif source == "stream":
            self.stream_frame_count += 1
            print(
                f"[REC] Stream frame #{self.stream_frame_count} "
                f"-> total #{len(self.frames)} "
                f"(joints={len(frame)}, duration={dur:.3f}s)"
            )

        return True

    def start_stream(self, duration_sec: Optional[float] = None):
        if self.stream_active:
            print("[INFO] Stream recording is already active.")
            return

        now = time.time()
        self.stream_active = True
        self.stream_until = None if duration_sec is None else now + float(duration_sec)
        self.last_stream_capture_time = None
        self.stream_frame_count = 0

        if duration_sec is None:
            print(f"[REC] Stream recording started. Interval={self.stream_interval:.3f}s. Use 'stop' to stop.")
        else:
            print(f"[REC] Stream recording started for {duration_sec:.2f}s. Interval={self.stream_interval:.3f}s.")

    def stop_stream(self):
        if not self.stream_active:
            print("[INFO] Stream recording is not active.")
            return

        self.stream_active = False
        self.stream_until = None
        self.last_stream_capture_time = None

        print(f"[REC] Stream recording stopped. Stream frames captured: {self.stream_frame_count}")

    def toggle_stream(self):
        if self.stream_active:
            self.stop_stream()
        else:
            self.start_stream()

    def tick_stream(self):
        if not self.stream_active:
            return

        now = time.time()

        if self.stream_until is not None and now >= self.stream_until:
            self.stop_stream()
            return

        if self.last_stream_capture_time is None:
            duration = self.stream_interval
            self.last_stream_capture_time = now
        else:
            elapsed = now - self.last_stream_capture_time
            if elapsed < self.stream_interval:
                return
            duration = elapsed
            self.last_stream_capture_time = now

        full = not self.stream_sparse

        self.capture_keyframe(
            full=full,
            duration=duration,
            source="stream",
            allow_empty=False,
        )

    def clear_buffer(self):
        self.frames = []
        self.prev_saved_pose = None
        self.stream_frame_count = 0
        self.saved_path = None
        print("[OK] Buffer cleared. Next recording will start from a new FULL keyframe.")

    def save_trajectory(
        self,
        filename_override: Optional[str] = None,
        *,
        clear_after_save: bool = False,
    ) -> Optional[str]:
        if not self.frames:
            print("[WARN] No keyframes to save.")
            return None

        os.makedirs(self.output_dir, exist_ok=True)

        if filename_override:
            filename = filename_override.strip()
            if not filename.endswith(".json"):
                filename += ".json"
        elif self.output_name:
            filename = self.output_name
            if not filename.endswith(".json"):
                filename += ".json"
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"keyframes_{timestamp}.json"

        if os.path.isabs(filename):
            path = filename
        else:
            path = os.path.join(self.output_dir, filename)

        os.makedirs(os.path.dirname(path), exist_ok=True)

        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.frames, f, ensure_ascii=False, indent=4)

        total_duration = sum(float(x.get("duration", 0.0)) for x in self.frames)
        self.saved_path = path

        print(f"[OK] Saved: {path}")
        print(f"[OK] Keyframes: {len(self.frames)}")
        print(f"[OK] Total dataset duration: {total_duration:.2f}s")

        if clear_after_save:
            self.clear_buffer()

        return path

    def signal_handler(self, sig, frame):
        if not self.running:
            return

        self.running = False
        print("\n[INFO] Stopping recorder...")

        if self.stream_active:
            self.stop_stream()

        self.save_trajectory(clear_after_save=False)
        self.close()
        sys.exit(0)

    def set_default_duration(self, duration: float):
        self.default_duration = max(0.01, float(duration))
        print(f"[OK] Default duration set to {self.default_duration:.3f}s")

    def print_status(self):
        with self.lock:
            has_pose = self.latest_pose is not None
            msg_count = self.msg_count
            latest_pose_len = len(self.latest_pose or {})
            latest_vel = dict(self.latest_vel or {})

        max_vel = max((abs(v) for v in latest_vel.values()), default=0.0)

        print(
            f"[STATUS] lowstate_received={has_pose}, "
            f"msg_count={msg_count}, "
            f"pose_joints={latest_pose_len}, "
            f"captured_keyframes={len(self.frames)}, "
            f"default_duration={self.default_duration:.3f}s, "
            f"stream_active={self.stream_active}, "
            f"stream_interval={self.stream_interval:.3f}s, "
            f"stream_frames={self.stream_frame_count}, "
            f"max_abs_vel={max_vel:.2f} deg/s"
        )


def build_controlled_joints(args) -> List[str]:
    joints: List[str] = []

    if args.side in {"both", "left"}:
        joints += LEFT_ARM_BASE_JOINTS
        if args.enable_wrist_py:
            joints += LEFT_WRIST_PY_JOINTS

    if args.side in {"both", "right"}:
        joints += RIGHT_ARM_BASE_JOINTS
        if args.enable_wrist_py:
            joints += RIGHT_WRIST_PY_JOINTS

    if args.enable_waist:
        joints += WAIST_JOINTS

    if args.enable_legs:
        joints += LEG_JOINTS

    seen = set()
    out = []
    for j in joints:
        if j not in seen:
            out.append(j)
            seen.add(j)

    return out


def maybe_auto_stable_capture(
    recorder: KeyframeRecorder,
    *,
    dwell: float,
    vel_eps: float,
    state: Dict,
):
    if recorder.stream_active:
        return

    with recorder.lock:
        latest_vel = dict(recorder.latest_vel or {})
        has_pose = recorder.latest_pose is not None

    if not has_pose or not latest_vel:
        state["stable_since"] = None
        return

    max_vel = max(abs(v) for v in latest_vel.values())
    now = time.time()

    if max_vel <= vel_eps:
        if state.get("stable_since") is None:
            state["stable_since"] = now

        stable_time = now - state["stable_since"]

        if stable_time >= dwell:
            if now - state.get("last_capture_time", 0.0) >= dwell:
                recorder.capture_keyframe(full=False, source="manual")
                state["last_capture_time"] = now
                state["stable_since"] = None
    else:
        state["stable_since"] = None


def parse_filename_after_command(cmd: str) -> Optional[str]:
    parts = cmd.split(maxsplit=1)
    if len(parts) > 1:
        name = parts[1].strip()
        return name if name else None
    return None


def parse_command(line: str, recorder: KeyframeRecorder) -> bool:
    cmd = line.strip()

    if cmd == "":
        recorder.capture_keyframe(full=False, source="manual")
        return True

    lower = cmd.lower()

    if lower == "enter" or lower.startswith("enter "):
        parts = lower.split()

        if len(parts) == 1:
            delay = 0.0
        else:
            try:
                delay = max(0.0, float(parts[1]))
            except Exception:
                print("[WARN] Usage: enter 5")
                return True

        if delay > 0:
            print(f"[INFO] Will capture keyframe in {delay:.1f}s...")
            time.sleep(delay)

        recorder.capture_keyframe(full=False, source="manual")
        return True

    if lower in {"f", "full"}:
        recorder.capture_keyframe(full=True, source="manual")
        return True

    if lower.startswith("d "):
        try:
            new_duration = float(cmd.split(maxsplit=1)[1])
            recorder.set_default_duration(new_duration)
        except Exception:
            print("[WARN] Usage: d 0.8")
        return True

    if lower in {"r", "rec", "stream"}:
        recorder.toggle_stream()
        return True

    if lower.startswith("record"):
        parts = lower.split()
        if len(parts) == 1:
            recorder.start_stream(duration_sec=None)
            return True

        try:
            duration_sec = float(parts[1])
            recorder.start_stream(duration_sec=duration_sec)
        except Exception:
            print("[WARN] Usage: record 5")
        return True

    if lower in {"stop", "stoprec", "stop_record", "stop-record"}:
        recorder.stop_stream()
        return True

    if lower == "s" or lower.startswith("s "):
        if recorder.stream_active:
            recorder.stop_stream()

        filename = parse_filename_after_command(cmd)
        recorder.save_trajectory(filename_override=filename, clear_after_save=True)
        return True

    if lower == "save" or lower.startswith("save "):
        if recorder.stream_active:
            recorder.stop_stream()

        filename = parse_filename_after_command(cmd)
        recorder.save_trajectory(filename_override=filename, clear_after_save=True)
        return True

    if lower in {"status", "st"}:
        recorder.print_status()
        return True

    if lower == "clear" or lower == "reset":
        recorder.clear_buffer()
        return True

    if lower == "q" or lower.startswith("q ") or lower == "quit" or lower.startswith("quit ") or lower == "exit" or lower.startswith("exit "):
        if recorder.stream_active:
            recorder.stop_stream()

        filename = parse_filename_after_command(cmd)
        recorder.running = False
        recorder.save_trajectory(filename_override=filename, clear_after_save=False)
        recorder.close()
        print("[INFO] Bye.")
        return False

    print("[WARN] Unknown command.")
    print("       Enter | enter 5 | f | d 0.8 | r | record 5 | stop | s name.json | status | clear | q")
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Record Unitree G1 keyframes / stream frames from LowState into JSON motion-plan format."
    )

    parser.add_argument("network_interface", nargs="?", default="eth0")
    parser.add_argument("--topic", default="rt/lowstate")

    parser.add_argument(
        "--output-dir",
        default="/home/unitree/unitree_sdk2_python/example/g1/high_level/data_steer/data_real_keyframes",
    )
    parser.add_argument(
        "--output-name",
        default=None,
        help="Default output filename. Can be overridden interactively: s name.json",
    )

    parser.add_argument("--duration", type=float, default=0.6, help="Default duration per manual captured keyframe.")
    parser.add_argument("--angle-eps", type=float, default=1.0, help="Sparse threshold in degrees.")
    parser.add_argument("--round-digits", type=int, default=1, help="Round angles to N decimals.")
    parser.add_argument("--min-changed-joints", type=int, default=1)
    parser.add_argument("--no-sparse", action="store_true", help="Manual keyframes are always full controlled pose.")

    parser.add_argument("--stream-interval", type=float, default=0.2, help="Continuous recording interval in seconds.")
    parser.add_argument(
        "--stream-sparse",
        action="store_true",
        help="Stream recording writes sparse frames. Default: stream writes full frames.",
    )

    parser.add_argument(
        "--side",
        choices=["both", "left", "right"],
        default="both",
        help="Which arm side to record.",
    )
    parser.add_argument("--enable-wrist-py", action="store_true", default=True, help="Record wrist pitch/yaw joints.")
    parser.add_argument("--disable-wrist-py", dest="enable_wrist_py", action="store_false", help="Do not record wrist pitch/yaw joints.")
    parser.add_argument("--enable-waist", action="store_true", help="Record waist joints.")
    parser.add_argument("--enable-legs", action="store_true", help="Record leg joints. Usually not needed for stage 1.")

    parser.add_argument(
        "--mode",
        choices=["manual", "stable"],
        default="manual",
        help="manual: Enter captures. stable: auto-capture when joints stop moving. Commands still work in both modes.",
    )
    parser.add_argument("--stable-dwell", type=float, default=0.5, help="Stable duration before auto capture.")
    parser.add_argument("--stable-vel-eps", type=float, default=2.0, help="Velocity threshold in deg/s for stable mode.")

    args = parser.parse_args()

    controlled_joints = build_controlled_joints(args)

    if not controlled_joints:
        raise ValueError("No controlled joints selected. Check --side / --enable-* flags.")

    ChannelFactoryInitialize(0, args.network_interface)

    recorder = KeyframeRecorder(
        topic=args.topic,
        output_dir=args.output_dir,
        output_name=args.output_name,
        controlled_sdk_names=controlled_joints,
        default_duration=args.duration,
        angle_eps=args.angle_eps,
        round_digits=args.round_digits,
        no_sparse=args.no_sparse,
        min_changed_joints=args.min_changed_joints,
        stream_interval=args.stream_interval,
        stream_sparse=args.stream_sparse,
    )

    recorder.start()

    print(f"[INFO] Interface: {args.network_interface}")
    print(f"[INFO] Mode: {args.mode}")
    print(f"[INFO] Side: {args.side}")
    print(f"[INFO] Output dir: {args.output_dir}")
    print(f"[INFO] Stream interval: {args.stream_interval:.3f}s")
    print(f"[INFO] Stream format: {'SPARSE' if args.stream_sparse else 'FULL'}")

    stable_state = {
        "stable_since": None,
        "last_capture_time": 0.0,
    }

    try:
        while recorder.running:
            recorder.tick_stream()

            if args.mode == "stable":
                maybe_auto_stable_capture(
                    recorder,
                    dwell=args.stable_dwell,
                    vel_eps=args.stable_vel_eps,
                    state=stable_state,
                )

            ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            if ready:
                line = sys.stdin.readline()
                should_continue = parse_command(line, recorder)
                if not should_continue:
                    return

    except KeyboardInterrupt:
        recorder.signal_handler(None, None)


if __name__ == "__main__":
    main()
