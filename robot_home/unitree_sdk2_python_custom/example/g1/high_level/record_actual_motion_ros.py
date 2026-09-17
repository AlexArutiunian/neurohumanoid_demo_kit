#!/usr/bin/env python3
import argparse
import csv
import json
import math
import os
import signal
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import JointState
import tf2_ros


DEFAULT_LINKS = [
    "pelvis",
    "torso_link",

    "left_shoulder_pitch_link",
    "left_shoulder_roll_link",
    "left_shoulder_yaw_link",
    "left_elbow_link",
    "left_wrist_roll_link",
    "left_wrist_pitch_link",
    "left_wrist_yaw_link",

    "right_shoulder_pitch_link",
    "right_shoulder_roll_link",
    "right_shoulder_yaw_link",
    "right_elbow_link",
    "right_wrist_roll_link",
    "right_wrist_pitch_link",
    "right_wrist_yaw_link",
]


class ActualMotionRecorder(Node):
    def __init__(self, args):
        super().__init__("actual_motion_recorder")

        self.args = args
        self.out_dir = Path(args.out_dir).expanduser().resolve() / args.run_name
        self.out_dir.mkdir(parents=True, exist_ok=True)

        self.target_frame = args.target_frame
        self.links = [x.strip() for x in args.links.split(",") if x.strip()]
        if not self.links:
            self.links = DEFAULT_LINKS

        self.t0_wall = time.time()
        self.last_joint_msg = None
        self.joint_header_written = False
        self.links_header_written = False

        self.angles_csv_path = self.out_dir / "actual_joint_angles_wide.csv"
        self.links_csv_path = self.out_dir / "actual_links_xyz_wide.csv"
        self.meta_path = self.out_dir / "meta.json"

        self.angles_f = open(self.angles_csv_path, "w", newline="", encoding="utf-8")
        self.links_f = open(self.links_csv_path, "w", newline="", encoding="utf-8")

        self.angles_writer = csv.writer(self.angles_f)
        self.links_writer = csv.writer(self.links_f)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            JointState,
            args.joint_topic,
            self.on_joint_state,
            50,
        )

        period = 1.0 / max(1.0, float(args.rate_hz))
        self.timer = self.create_timer(period, self.on_timer)

        meta = {
            "run_name": args.run_name,
            "created_wall_time": self.t0_wall,
            "joint_topic": args.joint_topic,
            "target_frame": self.target_frame,
            "links": self.links,
            "rate_hz": args.rate_hz,
            "out_dir": str(self.out_dir),
        }
        self.meta_path.write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")

        self.get_logger().info(f"Recording to: {self.out_dir}")
        self.get_logger().info(f"Angles: {self.angles_csv_path}")
        self.get_logger().info(f"XYZ:    {self.links_csv_path}")

    def on_joint_state(self, msg):
        self.last_joint_msg = msg

    def now_fields(self):
        t_wall = time.time()
        t_rel = t_wall - self.t0_wall
        t_ros = self.get_clock().now().nanoseconds / 1e9
        return t_wall, t_rel, t_ros

    def on_timer(self):
        t_wall, t_rel, t_ros = self.now_fields()

        self.write_joint_angles(t_wall, t_rel, t_ros)
        self.write_links_xyz(t_wall, t_rel, t_ros)

    def write_joint_angles(self, t_wall, t_rel, t_ros):
        msg = self.last_joint_msg
        if msg is None or not msg.name:
            return

        names = list(msg.name)
        positions = list(msg.position)

        if not self.joint_header_written:
            header = ["t_wall", "t_rel", "t_ros"]
            for name in names:
                header.append(f"{name}_rad")
                header.append(f"{name}_deg")
            self.angles_writer.writerow(header)
            self.joint_header_written = True

        row = [f"{t_wall:.6f}", f"{t_rel:.6f}", f"{t_ros:.6f}"]
        for q in positions:
            row.append(f"{q:.9f}")
            row.append(f"{math.degrees(q):.6f}")
        self.angles_writer.writerow(row)
        self.angles_f.flush()

    def write_links_xyz(self, t_wall, t_rel, t_ros):
        if not self.links_header_written:
            header = ["t_wall", "t_rel", "t_ros"]
            for link in self.links:
                header += [
                    f"{link}_x",
                    f"{link}_y",
                    f"{link}_z",
                    f"{link}_qx",
                    f"{link}_qy",
                    f"{link}_qz",
                    f"{link}_qw",
                    f"{link}_ok",
                ]
            self.links_writer.writerow(header)
            self.links_header_written = True

        row = [f"{t_wall:.6f}", f"{t_rel:.6f}", f"{t_ros:.6f}"]

        for link in self.links:
            try:
                tr = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    link,
                    Time(),
                    timeout=rclpy.duration.Duration(seconds=0.01),
                )
                p = tr.transform.translation
                q = tr.transform.rotation
                row += [
                    f"{p.x:.9f}",
                    f"{p.y:.9f}",
                    f"{p.z:.9f}",
                    f"{q.x:.9f}",
                    f"{q.y:.9f}",
                    f"{q.z:.9f}",
                    f"{q.w:.9f}",
                    "1",
                ]
            except Exception:
                row += ["", "", "", "", "", "", "", "0"]

        self.links_writer.writerow(row)
        self.links_f.flush()

    def close_files(self):
        try:
            self.angles_f.flush()
            self.angles_f.close()
        except Exception:
            pass
        try:
            self.links_f.flush()
            self.links_f.close()
        except Exception:
            pass


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--out-dir", default="./actual_motion_logs")
    p.add_argument("--run-name", required=True)
    p.add_argument("--joint-topic", default="/joint_states")
    p.add_argument("--target-frame", default="pelvis")
    p.add_argument("--rate-hz", type=float, default=50.0)
    p.add_argument(
        "--links",
        default=",".join(DEFAULT_LINKS),
        help="Comma-separated TF link names to record",
    )
    args = p.parse_args()

    rclpy.init()
    node = ActualMotionRecorder(args)

    stop = {"flag": False}

    def _sig_handler(signum, frame):
        stop["flag"] = True

    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    try:
        while rclpy.ok() and not stop["flag"]:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.close_files()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
