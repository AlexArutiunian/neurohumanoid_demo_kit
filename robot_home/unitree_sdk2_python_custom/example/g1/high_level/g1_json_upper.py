#!/usr/bin/env python3
import time
import sys
import os
import json
import math
import select
import threading

"""

push button 

python3 g1_json_upper.py --dataset-path ./216.json --enable-wrist-py



python3 g1_json_hands.py --dataset-path ./216.json --enable-wrist-py --kp-waist 250 --kd-waist 6

python3 g1_json_hands.py --dataset-path ./push_button_17feb.json --enable-wrist-py --kp-waist 250 --kd-waist 6


python3 g1_json_hands.py --dataset-path ./pipeline/ik_records/current_target_right.json
 --enable-wrist-py --kp-waist 250 --kd-waist 6

python3 g1_json_upper.py \
  --dataset-path ./waist_yaw_test_robot.json \
  --enable-waist
  
python3 g1_json_upper.py --dataset-path ./waist_yaw_left_45.json --enable-waist --enable-wrist-py
  
python3 g1_json_upper.py --dataset-path ./replace_box.json --enable-waist --enable-wrist-py
  
python3 g1_json_upper.py --dataset-path ./left_to_grab.json --enable-waist --enable-wrist-py

python3 g1_json_upper.py eth0 --dataset-path ./left_to_grab.json --enable-wrist-py --kp-waist 250 --kd-waist 6 --dataset-duration-max 120 --skip-empty-on-enter

left_hand_up.json

python3 g1_json_upper.py eth0 --dataset-path motions/left_hand_up.json --enable-wrist-py --kp-waist 250 --kd-waist 6 --dataset-duration-max 120 --skip-empty-on-enter

python3 g1_json_upper.py eth0 --dataset-path pipeline/dataset/1.json --enable-wrist-py --kp-waist 250 --kd-waist 6 --dataset-duration-max 120 --skip-empty-on-enter --enable-waist

python3 g1_json_upper.py eth0 --dataset-path pipeline/dataset/1.json --enable-wrist-py --kp-waist 800 --kd-waist 15 --dataset-duration-max 120 --skip-empty-on-enter --enable-waist

python3 g1_json_upper.py eth0 --dataset-path pipeline/dataset/data_steer/1.json --enable-wrist-py --kp-waist 250 --kd-waist 6 --dataset-duration-max 120 --skip-empty-on-enter --enable-waist

python3 g1_json_upper.py eth0 --dataset-path dataset_100/47.json --enable-wrist-py --kp-waist 250 --kd-waist 6 --dataset-duration-max 120 --skip-empty-on-enter --enable-waist


"""

import numpy as np

from unitree_sdk2py.core.channel import (
    ChannelPublisher,
    ChannelSubscriber,
    ChannelFactoryInitialize,
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread


class G1JointIndex:
    # Legs
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleRoll = 5

    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleRoll = 11

    # Waist
    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14

    # Arms
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21

    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28

    # Weight: 1 enable arm_sdk, 0 disable
    kNotUsedJoint = 29


DEG2RAD = math.pi / 180.0

WAIST_JOINTS = (G1JointIndex.WaistYaw, G1JointIndex.WaistRoll, G1JointIndex.WaistPitch)

JOINT_NAME_TO_INDEX = {
    "left_shoulder_pitch_joint":  G1JointIndex.LeftShoulderPitch,
    "left_shoulder_roll_joint":   G1JointIndex.LeftShoulderRoll,
    "left_shoulder_yaw_joint":    G1JointIndex.LeftShoulderYaw,
    "left_elbow_joint":           G1JointIndex.LeftElbow,
    "left_wrist_roll_joint":      G1JointIndex.LeftWristRoll,
    "left_wrist_pitch_joint":     G1JointIndex.LeftWristPitch,
    "left_wrist_yaw_joint":       G1JointIndex.LeftWristYaw,

    "right_shoulder_pitch_joint": G1JointIndex.RightShoulderPitch,
    "right_shoulder_roll_joint":  G1JointIndex.RightShoulderRoll,
    "right_shoulder_yaw_joint":   G1JointIndex.RightShoulderYaw,
    "right_elbow_joint":          G1JointIndex.RightElbow,
    "right_wrist_roll_joint":     G1JointIndex.RightWristRoll,
    "right_wrist_pitch_joint":    G1JointIndex.RightWristPitch,
    "right_wrist_yaw_joint":      G1JointIndex.RightWristYaw,

    # waist_* оставляем в словаре (чтобы можно было детектить и предупреждать),
    # но управлять ими из датасета НЕ будем — талия фиксируется в стартовых углах.
    "waist_yaw_joint":            G1JointIndex.WaistYaw,
    "waist_roll_joint":           G1JointIndex.WaistRoll,
    "waist_pitch_joint":          G1JointIndex.WaistPitch,
}

def expand_repeats(data):
    if isinstance(data, dict):
        data = [data]

    if not isinstance(data, list):
        raise ValueError("Dataset JSON must be a list/dict of frame/repeat blocks")

    out = []
    for block in data:
        if not isinstance(block, dict):
            continue

        if "frame" in block:
            out.append(block)
            continue

        if "repeat" in block:
            # формат: {"repeat": 4, "sequence": [...]}
            if isinstance(block["repeat"], int):
                times = int(block["repeat"])
                inner = block.get("sequence", [])
            # формат: {"repeat": [...], "times": 4}
            else:
                times = int(block.get("times", 1))
                inner = block["repeat"]

            inner = expand_repeats(inner)
            for _ in range(max(0, times)):
                out.extend(inner)

    return out

class Custom:
    def __init__(self, *, enable_wrist_pitch_yaw=False, enable_waist=False):
        self.time_ = 0.0
        self.control_dt_ = 0.02

        # Параметры рук
        self.kp = 60.0
        self.kd = 1.5

        # Параметры талии (держим корпус, но без “бетона”)
        self.kp_waist = 250.0
        self.kd_waist = 6

        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.first_update_low_state = False
        self.crc = CRC()
        self.done = False
        self.time_lock = threading.Lock()

        # Мягкая передача/сдача arm_sdk
        self.weight_ramp = 1.5

        # Возврат в стартовую позу и холд перед сдачей управления
        self.return_duration = 2.0
        self.hold_duration = 1.5

        self.enable_wrist_pitch_yaw_ = bool(enable_wrist_pitch_yaw)
        self.enable_waist_ = bool(enable_waist)

        # Суставы, которые ведёт траектория (только руки)
        self.arm_joints = [
            G1JointIndex.LeftShoulderPitch, G1JointIndex.LeftShoulderRoll,
            G1JointIndex.LeftShoulderYaw,   G1JointIndex.LeftElbow,
            G1JointIndex.LeftWristRoll,
        ]
        if self.enable_wrist_pitch_yaw_:
            self.arm_joints += [G1JointIndex.LeftWristPitch, G1JointIndex.LeftWristYaw]

        self.arm_joints += [
            G1JointIndex.RightShoulderPitch, G1JointIndex.RightShoulderRoll,
            G1JointIndex.RightShoulderYaw,   G1JointIndex.RightElbow,
            G1JointIndex.RightWristRoll,
        ]
        if self.enable_wrist_pitch_yaw_:
            self.arm_joints += [G1JointIndex.RightWristPitch, G1JointIndex.RightWristYaw]

        self.joint_to_pose_i_ = {j: i for i, j in enumerate(self.arm_joints)}
        self.waist_to_pose_i_ = {j: i for i, j in enumerate(WAIST_JOINTS)}

        # Стартовая поза рук (из lowstate)
        self.q_start_arms = None
        self.q_start_waist = None
        # Фиксация талии в углах “как было до старта”
        self.waist_hold_q = None  # dict: joint -> q(rad)

        # Keyframes по умолчанию (если не загрузили JSON)
        self.keyframes = [
            {"frame": [{"name": "left_shoulder_pitch_joint", "angle": -10},
                       {"name": "right_shoulder_pitch_joint", "angle": -10}],
             "duration": 0.8},
        ]

        self.poses = None
        self.waist_poses = None
        self.empty_segments = set()
        self.skip_current_empty_requested = False
        self.durations = [max(0.05, float(kf.get("duration", 0.6))) for kf in self.keyframes]
        self.total_time = sum(self.durations)

    def LoadDataset(
        self,
        dataset_path,
        *,
        ignore_duration=False,
        fixed_duration=None,
        duration_scale=1.0,
        duration_min=0.01,
        duration_max=20.0,
        reset_timing=True,
    ):
        if not os.path.isfile(dataset_path):
            raise FileNotFoundError(dataset_path)

        with open(dataset_path, "r", encoding="utf-8") as f:
    	    data = json.load(f)

        data = expand_repeats(data)
        print(f"[JSON] expanded keyframes: {len(data)}")

        if not isinstance(data, list):
            raise ValueError("Dataset JSON must be a list of {frame, duration} objects")

        if fixed_duration is None:
            fixed_duration = 0.6

        keyframes_raw = []
        ignored = set()

        for step in data:
            if not isinstance(step, dict):
                continue

            dr = step.get("duration", fixed_duration)
            duration_raw = float(fixed_duration if dr is None else dr)
            duration = duration_raw * float(duration_scale)

            if ignore_duration:
                duration = float(fixed_duration)
            else:
                if duration_min is not None:
                    duration = max(float(duration_min), duration)
                if duration_max is not None:
                    duration = min(float(duration_max), duration)

            duration = max(0.05, duration)

            frame = step.get("frame", [])
            if not isinstance(frame, list):
                frame = []

            # Предупреждения по суставам
            for item in frame:
                if isinstance(item, dict) and isinstance(item.get("name"), str):
                    name = item["name"]
                    if name not in JOINT_NAME_TO_INDEX:
                        ignored.add(name)
                    else:
                        jidx = JOINT_NAME_TO_INDEX[name]
                        # Талия из датасета управляется только при явном флаге.
                        if jidx in WAIST_JOINTS:
                            if not self.enable_waist_:
                                ignored.add(name)
                        # руки: только если сустав реально в arm_joints
                        elif jidx not in self.arm_joints:
                            ignored.add(name)

            keyframes_raw.append({"frame": frame, "duration": duration})

        if not keyframes_raw:
            raise ValueError("No keyframes parsed from dataset: " + dataset_path)

        self.keyframes = keyframes_raw
        self.durations = [kf["duration"] for kf in self.keyframes]
        self.total_time = sum(self.durations)
        self.empty_segments = {
            i for i, kf in enumerate(self.keyframes) if len(kf.get("frame", [])) == 0
        }

        # чтобы пересобралось на первом LowStateHandler
        self.q_start_arms = None
        self.q_start_waist = None
        self.poses = None
        self.waist_poses = None

        if ignored:
            reason = "unknown / not controlled"
            if not self.enable_waist_:
                reason += " / waist locked by hold"
            print(
                f"WARNING: Some joints from dataset are ignored ({reason}):\n"
                + "\n".join(sorted(ignored))
            )

        if reset_timing:
            with self.time_lock:
                self.time_ = 0.0
            self.done = False
            self.skip_current_empty_requested = False

    def RequestSkipCurrentEmpty(self):
        self.skip_current_empty_requested = True

    def _segment_index_and_cum(self, t):
        cum = 0.0
        seg = 0
        for i, dur in enumerate(self.durations):
            if cum + dur > t:
                seg = i
                break
            cum += dur
        return seg, cum

    def Init(self):
        self.arm_sdk_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_sdk_publisher.Init()

        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )
        while not self.first_update_low_state:
            time.sleep(0.2)
        self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg
        if not self.first_update_low_state:
            self.first_update_low_state = True

        # 1) Один раз: фиксируем “как было” по талии и старт рук
        if self.waist_hold_q is None:
            self.waist_hold_q = {
                G1JointIndex.WaistYaw:   msg.motor_state[G1JointIndex.WaistYaw].q,
                G1JointIndex.WaistRoll:  msg.motor_state[G1JointIndex.WaistRoll].q,
                G1JointIndex.WaistPitch: msg.motor_state[G1JointIndex.WaistPitch].q,
            }

        if self.q_start_arms is None:
            self.q_start_arms = [msg.motor_state[j].q for j in self.arm_joints]
            self.q_start_waist = [msg.motor_state[j].q for j in WAIST_JOINTS]

            pose_prev = list(self.q_start_arms)
            poses = [pose_prev]
            waist_prev = list(self.q_start_waist)
            waist_poses = [waist_prev]

            # 2) Собираем poses из keyframes (руки + талия при --enable-waist)
            for kf in self.keyframes:
                pose_next = pose_prev[:]
                pose_next_waist = waist_prev[:]
                for item in kf.get("frame", []):
                    if not isinstance(item, dict):
                        continue
                    name = item.get("name")
                    if not isinstance(name, str) or name not in JOINT_NAME_TO_INDEX:
                        continue
                    joint_idx = JOINT_NAME_TO_INDEX[name]

                    if joint_idx in WAIST_JOINTS:
                        if self.enable_waist_:
                            waist_i = self.waist_to_pose_i_.get(joint_idx)
                            if waist_i is not None:
                                angle_deg = item.get("angle", 0.0)
                                pose_next_waist[waist_i] = float(angle_deg) * DEG2RAD
                        continue

                    pose_i = self.joint_to_pose_i_.get(joint_idx)
                    if pose_i is None:
                        continue
                    angle_deg = item.get("angle", 0.0)
                    pose_next[pose_i] = float(angle_deg) * DEG2RAD

                poses.append(pose_next)
                waist_poses.append(pose_next_waist)
                pose_prev = pose_next
                waist_prev = pose_next_waist

            # 3) Добавляем плавный возврат рук в старт и холд
            return_dur = max(0.05, float(self.return_duration))
            hold_dur = max(0.05, float(self.hold_duration))

            poses.append(list(self.q_start_arms))
            self.durations.append(return_dur)
            waist_poses.append(list(self.q_start_waist))

            poses.append(list(self.q_start_arms))
            self.durations.append(hold_dur)
            waist_poses.append(list(self.q_start_waist))

            self.total_time = sum(self.durations)
            self.poses = poses
            self.waist_poses = waist_poses

    def _apply_waist_targets(self, q_waist=None):
        """Талия либо идет по траектории, либо удерживается в стартовых углах."""
        if self.low_state is None or self.waist_hold_q is None:
            return
        q_cmd = q_waist if q_waist is not None else [self.waist_hold_q[j] for j in WAIST_JOINTS]
        for j in WAIST_JOINTS:
            wi = self.waist_to_pose_i_[j]
            self.low_cmd.motor_cmd[j].tau = 0.0
            self.low_cmd.motor_cmd[j].dq = 0.0
            self.low_cmd.motor_cmd[j].q = float(q_cmd[wi])
            self.low_cmd.motor_cmd[j].kp = self.kp_waist
            self.low_cmd.motor_cmd[j].kd = self.kd_waist

    def LowCmdWrite(self):
        if self.low_state is None or self.poses is None or self.waist_hold_q is None:
            return
        if self.enable_waist_ and self.waist_poses is None:
            return

        with self.time_lock:
            self.time_ += self.control_dt_
            t = self.time_

        # 1) weight arm_sdk (включение/выключение мягко)
        if t < self.weight_ramp:
            w = np.clip(t / self.weight_ramp, 0.0, 1.0)
        elif t >= self.total_time:
            hold_until = self.total_time + self.weight_ramp
            if t >= hold_until:
                self.done = True
                self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 0.0
                q_waist_final = self.waist_poses[-1] if self.enable_waist_ else None
                self._apply_waist_targets(q_waist_final)
                self.low_cmd.crc = self.crc.Crc(self.low_cmd)
                self.arm_sdk_publisher.Write(self.low_cmd)
                return
            w = np.clip((hold_until - t) / self.weight_ramp, 0.0, 1.0)
        else:
            w = 1.0

        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = float(w)

        # 2) если анимация закончилась — держим последнюю позу рук, пока weight спадает
        if t >= self.total_time:
            q_final = self.poses[-1]
            q_waist_final = self.waist_poses[-1] if self.enable_waist_ else None
            for i, joint in enumerate(self.arm_joints):
                self.low_cmd.motor_cmd[joint].tau = 0.0
                self.low_cmd.motor_cmd[joint].dq = 0.0
                self.low_cmd.motor_cmd[joint].q = float(q_final[i])
                self.low_cmd.motor_cmd[joint].kp = self.kp
                self.low_cmd.motor_cmd[joint].kd = self.kd

            self._apply_waist_targets(q_waist_final)
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.arm_sdk_publisher.Write(self.low_cmd)
            return

        # 3) находим сегмент траектории
        seg, cum = self._segment_index_and_cum(t)

        # Skip only empty hold segment (e.g., [] frame in dataset).
        if self.skip_current_empty_requested:
            self.skip_current_empty_requested = False
            if seg in self.empty_segments:
                seg_dur_now = max(0.05, float(self.durations[seg]))
                with self.time_lock:
                    self.time_ = cum + seg_dur_now + 1e-4
                    t = self.time_
                seg, cum = self._segment_index_and_cum(t)
                print(f"[INFO] Skipped empty frame segment #{seg}")
            else:
                print("[INFO] Skip ignored: current segment is not an empty frame.")

        seg_t = t - cum
        seg_dur = max(0.05, float(self.durations[seg]))
        ratio = np.clip(seg_t / seg_dur, 0.0, 1.0)
        ratio = ratio * ratio * (3.0 - 2.0 * ratio)  # smoothstep

        q0 = self.poses[seg]
        q1 = self.poses[seg + 1]

        q_traj = np.array([(1.0 - ratio) * q0[i] + ratio * q1[i] for i in range(len(self.arm_joints))])
        q_waist_traj = None
        if self.enable_waist_:
            q0w = self.waist_poses[seg]
            q1w = self.waist_poses[seg + 1]
            q_waist_traj = np.array([(1.0 - ratio) * q0w[i] + ratio * q1w[i] for i in range(len(WAIST_JOINTS))])

        # 4) в начале — мягко примешиваем от текущего к траектории, чтобы не было рывка
        if t < self.weight_ramp:
            w_arm = np.clip(t / self.weight_ramp, 0.0, 1.0)
            w_arm = w_arm * w_arm * (3.0 - 2.0 * w_arm)
            q_current = np.array([self.low_state.motor_state[j].q for j in self.arm_joints])
            q_traj = (1.0 - w_arm) * q_current + w_arm * q_traj
            if self.enable_waist_ and q_waist_traj is not None:
                q_waist_current = np.array([self.low_state.motor_state[j].q for j in WAIST_JOINTS])
                q_waist_traj = (1.0 - w_arm) * q_waist_current + w_arm * q_waist_traj

        for i, joint in enumerate(self.arm_joints):
            self.low_cmd.motor_cmd[joint].tau = 0.0
            self.low_cmd.motor_cmd[joint].dq = 0.0
            self.low_cmd.motor_cmd[joint].q = float(q_traj[i])
            self.low_cmd.motor_cmd[joint].kp = self.kp
            self.low_cmd.motor_cmd[joint].kd = self.kd

        # 5) талия: траектория (если включено) или удержание старта.
        self._apply_waist_targets(q_waist_traj)

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.arm_sdk_publisher.Write(self.low_cmd)


if __name__ == "__main__":
    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    default_dataset_dir = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "../../../../../dataset")
    )

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("iface", nargs="?", default=None, help="Network interface (e.g. eth0). If omitted, uses default.")
    parser.add_argument("--dataset-id", type=int, default=None, help="Dataset number, e.g. 213 (loads <dataset-dir>/<id>.json)")
    parser.add_argument("--dataset-path", type=str, default=None, help="Explicit path to a dataset json file")
    parser.add_argument("--dataset-dir", type=str, default=default_dataset_dir, help="Dataset directory")
    parser.add_argument("--dataset-ignore-duration", action="store_true", help="Ignore duration in JSON; use fixed duration")
    parser.add_argument("--kf-duration", type=float, default=0.6, help="Fixed duration per keyframe if ignore-duration")
    parser.add_argument("--dataset-duration-scale", type=float, default=1.0, help="Multiply dataset durations")
    parser.add_argument("--dataset-duration-min", type=float, default=0.05, help="Clamp duration min")
    parser.add_argument("--dataset-duration-max", type=float, default=20.0, help="Clamp duration max")
    parser.add_argument("--enable-wrist-py", action="store_true", help="Enable wrist pitch/yaw joints (29dof only)")
    parser.add_argument("--enable-waist", action="store_true", help="Allow waist joints from dataset JSON")

    parser.add_argument("--kp", type=float, default=60.0, help="Arm kp")
    parser.add_argument("--kd", type=float, default=1.5, help="Arm kd")
    parser.add_argument("--kp-waist", type=float, default=250.0, help="Waist kp (hold)")
    parser.add_argument("--kd-waist", type=float, default=6.0, help="Waist kd (hold)")

    parser.add_argument("--return-duration", type=float, default=2.0, help="Return to start duration (s)")
    parser.add_argument("--hold-duration", type=float, default=1.5, help="Hold start duration (s)")
    parser.add_argument("--weight-ramp", type=float, default=1.5, help="Soft handover duration (s)")
    parser.add_argument(
        "--skip-empty-on-enter",
        action="store_true",
        help="Press Enter during run to skip current empty frame segment immediately.",
    )

    args = parser.parse_args()

    if args.iface is not None:
        ChannelFactoryInitialize(0, args.iface)
    else:
        ChannelFactoryInitialize(0)

    custom = Custom(enable_wrist_pitch_yaw=args.enable_wrist_py, enable_waist=args.enable_waist)
    custom.kp = float(args.kp)
    custom.kd = float(args.kd)
    custom.kp_waist = float(args.kp_waist)
    custom.kd_waist = float(args.kd_waist)
    custom.return_duration = float(args.return_duration)
    custom.hold_duration = float(args.hold_duration)
    custom.weight_ramp = float(args.weight_ramp)

    dataset_path = None
    if args.dataset_path:
        dataset_path = args.dataset_path
    elif args.dataset_id is not None:
        dataset_path = os.path.join(args.dataset_dir, f"{args.dataset_id}.json")

    if dataset_path is not None:
        print(f"Loading dataset: {dataset_path}")
        custom.LoadDataset(
            dataset_path,
            ignore_duration=args.dataset_ignore_duration,
            fixed_duration=args.kf_duration,
            duration_scale=args.dataset_duration_scale,
            duration_min=args.dataset_duration_min,
            duration_max=args.dataset_duration_max,
            reset_timing=True,
        )

    custom.Init()
    custom.Start()
    if args.skip_empty_on_enter:
        print("[INFO] Skip mode ON: press Enter to skip current empty frame segment.")

    while True:
        if args.skip_empty_on_enter:
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if ready:
                sys.stdin.readline()
                custom.RequestSkipCurrentEmpty()
        else:
            time.sleep(0.5)
        if custom.done:
            print("Done!")
            sys.exit(0)

