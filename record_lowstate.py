#!/usr/bin/env python3
"""WearBench-G1: пассивная запись Unitree G1 rt/lowstate в CSV"""

import argparse
import csv
import hashlib
import json
import queue
import re
import signal
import sys
import threading
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace

JOINT_NAMES = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]
FIELDS = ("q", "dq", "ddq", "tau_est", "temperature_0", "temperature_1",
          "vol", "mode", "motorstate")


def columns():
    head = ["wall_time_ns", "monotonic_ns", "sequence", "robot_tick",
            "mode_pr", "mode_machine"]
    head += ["imu_quaternion_" + str(i) for i in range(4)]
    head += ["imu_gyroscope_" + str(i) for i in range(3)]
    head += ["imu_accelerometer_" + str(i) for i in range(3)]
    head += ["imu_rpy_" + str(i) for i in range(3)]
    head += ["imu_temperature_raw"]
    for name in JOINT_NAMES:
        head += [name + "_" + field for field in FIELDS]
    return head


def make_row(msg, seq, wall_ns, mono_ns):
    if len(msg.motor_state) < len(JOINT_NAMES):
        raise ValueError("LowState has fewer than 29 motor states")
    imu = msg.imu_state
    row = [wall_ns, mono_ns, seq, int(msg.tick),
           int(msg.mode_pr), int(msg.mode_machine)]
    for key, expected in (("quaternion", 4), ("gyroscope", 3),
                          ("accelerometer", 3), ("rpy", 3)):
        values = getattr(imu, key)
        if len(values) != expected:
            raise ValueError("Unexpected IMU length: " + key)
        row.extend(float(x) for x in values)
    row.append(int(imu.temperature))
    for index in range(29):
        motor = msg.motor_state[index]
        if len(motor.temperature) != 2:
            raise ValueError("Unexpected motor temperature array")
        row.extend((float(motor.q), float(motor.dq), float(motor.ddq),
                    float(motor.tau_est), int(motor.temperature[0]),
                    int(motor.temperature[1]), float(motor.vol),
                    int(motor.mode), int(motor.motorstate)))
    return row


class Recorder:
    def __init__(self, csv_path, queue_size):
        self.path = csv_path
        self.items = queue.Queue(maxsize=queue_size)
        self.stop = threading.Event()
        self.first_ns = None
        self.last_ns = None
        self.received = 0
        self.queued = 0
        self.saved = 0
        self.dropped = 0
        self.decode_errors = 0
        self.writer_error = None
        self.thread = threading.Thread(target=self._write, name="wearbench_csv")

    def _write(self):
        try:
            with self.path.open("w", encoding="utf-8", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(columns())
                while not self.stop.is_set() or not self.items.empty():
                    try:
                        row = self.items.get(timeout=0.2)
                    except queue.Empty:
                        continue
                    writer.writerow(row)
                    self.saved += 1
                    if self.saved % 100 == 0:
                        f.flush()
                f.flush()
        except Exception as exc:
            self.writer_error = repr(exc)
            self.stop.set()

    def callback(self, msg):
        if self.stop.is_set():
            return
        wall_ns = time.time_ns()
        mono_ns = time.monotonic_ns()
        self.received += 1
        try:
            row = make_row(msg, self.received, wall_ns, mono_ns)
        except Exception:
            self.decode_errors += 1
            return
        if self.first_ns is None:
            self.first_ns = mono_ns
        self.last_ns = mono_ns
        try:
            self.items.put_nowait(row)
            self.queued += 1
        except queue.Full:
            self.dropped += 1


def _self_test():
    motor = SimpleNamespace(q=0.2, dq=-0.3, ddq=0.4, tau_est=1.2,
                            temperature=[31, 32], vol=48.0,
                            mode=1, motorstate=0)
    imu = SimpleNamespace(quaternion=[1, 0, 0, 0],
                          gyroscope=[0, 0, 0], accelerometer=[0, 0, 9.8],
                          rpy=[0, 0, 0], temperature=27)
    msg = SimpleNamespace(motor_state=[motor] * 35, imu_state=imu,
                          tick=123, mode_pr=0, mode_machine=1)
    row = make_row(msg, 1, 100000, 99900)
    assert len(row) == len(columns())
    assert row[columns().index("right_knee_joint_tau_est")] == 1.2
    assert JOINT_NAMES[9] == "right_knee_joint"
    with tempfile.TemporaryDirectory() as tmp:
        rec = Recorder(Path(tmp) / "test.csv", 10)
        rec.thread.start()
        rec.callback(msg)
        rec.stop.set()
        rec.thread.join(timeout=3)
        assert rec.writer_error is None and rec.saved == 1
        with (Path(tmp) / "test.csv").open(newline="") as f:
            saved = list(csv.reader(f))
        assert len(saved) == 2 and len(saved[1]) == len(columns())
    print("SELF-TEST OK: 29 joints, IMU, CSV writer, no SDK required")


def main():
    ap = argparse.ArgumentParser(description="G1 WearBench read-only LowState recorder")
    ap.add_argument("--iface", default="eth0", help="Network interface, e.g. eth0")
    ap.add_argument("--topic", default="rt/lowstate")
    ap.add_argument("--sdk-dir", default=None, help="Existing SDK2 source directory")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--startup-timeout", type=float, default=10.0)
    ap.add_argument("--label", default="stand")
    ap.add_argument("--out-dir", default="./wearbench_data")
    ap.add_argument("--queue-size", type=int, default=2048)
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    if args.self_test:
        _self_test()
        return 0
    if args.seconds <= 0 or args.startup_timeout <= 0 or args.queue_size <= 0:
        ap.error("seconds, startup-timeout and queue-size must be positive")
    if args.topic not in ("rt/lowstate", "rt/lf/lowstate"):
        ap.error("Only G1 read-only LowState topics are permitted")

    candidates = ([Path(args.sdk_dir).expanduser()] if args.sdk_dir else
                  [Path.home() / "unitree_sdk2_python_custom",
                   Path.home() / "unitree_sdk2_python"])
    sdk_root = next((p for p in candidates
                     if (p / "unitree_sdk2py" / "__init__.py").is_file()), None)
    if args.sdk_dir and sdk_root is None:
        print("SDK directory not found:", args.sdk_dir, file=sys.stderr)
        return 2
    if sdk_root is not None:
        sys.path.insert(0, str(sdk_root))
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    except ImportError as exc:
        print("SDK import failed:", exc, file=sys.stderr)
        print("Install the existing custom SDK dependencies; see README.md", file=sys.stderr)
        return 2

    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    label = re.sub(r"[^a-zA-Z0-9_-]", "_", args.label)[:48] or "run"
    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = out_dir / (stamp + "_" + label + ".csv")
    meta_path = csv_path.with_suffix(".meta.json")
    rec = Recorder(csv_path, args.queue_size)
    rec.thread.start()
    sub = None
    interrupted = threading.Event()

    def ask_stop(signum, frame):
        interrupted.set()

    old_int = signal.signal(signal.SIGINT, ask_stop)
    old_term = signal.signal(signal.SIGTERM, ask_stop)
    start = time.monotonic()
    reason = "duration"
    try:
        ChannelFactoryInitialize(0, args.iface)
        sub = ChannelSubscriber(args.topic, LowState_)
        sub.Init(rec.callback, 50)
        print("READ-ONLY:", args.topic, "iface:", args.iface)
        print("Saving:", csv_path)
        while not interrupted.is_set():
            time.sleep(0.2)
            if rec.writer_error:
                reason = "writer_error"
                break
            if rec.first_ns is None:
                if time.monotonic() - start > args.startup_timeout:
                    reason = "no_messages"
                    break
            elif (time.monotonic_ns() - rec.first_ns) >= args.seconds * 1e9:
                break
        if interrupted.is_set():
            reason = "interrupted"
    except Exception as exc:
        reason = "sdk_error"
        print("Recorder SDK error:", repr(exc), file=sys.stderr)
    finally:
        if sub is not None:
            try:
                sub.Close()
            except Exception as exc:
                print("Subscriber close warning:", repr(exc), file=sys.stderr)
        rec.stop.set()
        rec.thread.join(timeout=15)
        signal.signal(signal.SIGINT, old_int)
        signal.signal(signal.SIGTERM, old_term)

    meta = {
        "schema": "wearbench_g1_lowstate_v1",
        "label": label, "iface": args.iface, "topic": args.topic,
        "csv_file": csv_path.name, "duration_requested_s": args.seconds,
        "reason": reason, "received": rec.received, "queued": rec.queued,
        "saved": rec.saved, "queue_dropped": rec.dropped,
        "decode_errors": rec.decode_errors, "writer_error": rec.writer_error,
        "first_monotonic_ns": rec.first_ns, "last_monotonic_ns": rec.last_ns,
        "joint_names": JOINT_NAMES,
        "sdk_root": str(sdk_root) if sdk_root is not None else "installed_sdk",
        "recorder_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "notes": "tau_est is SDK-estimated, not calibrated shaft torque; two temperature fields are raw SDK values; no commands published",
    }
    meta_path.write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print("Finished:", json.dumps({k: meta[k] for k in
          ("reason", "received", "saved", "queue_dropped", "decode_errors",
           "writer_error")}, ensure_ascii=False))
    if rec.thread.is_alive() or rec.writer_error or rec.saved == 0:
        return 3
    if rec.dropped or rec.decode_errors:
        print("WARNING: incomplete dataset; inspect metadata before use")
        return 4
    return 0 if reason in ("duration", "interrupted") else 5


if __name__ == "__main__":
    sys.exit(main())
