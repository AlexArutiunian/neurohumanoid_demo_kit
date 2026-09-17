#!/usr/bin/env python3
import argparse
import logging
import math
import os
import select
import sys
import time
from dataclasses import dataclass
from typing import List, Optional

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

def _prepend_dftp_arm_candidates():
    here = os.path.dirname(__file__)
    repo_root = os.path.abspath(os.path.join(here, "../../../../"))
    user_home = os.path.expanduser("~")
    candidates = [
        os.environ.get("DFTP_ARM_PATH", ""),
        os.path.join(repo_root, "DFTP_arm"),
        os.path.abspath(os.path.join(here, "../../../../../DFTP_arm")),
        os.path.join(user_home, "DFTP_arm"),
        "/home/unitree/DFTP_arm",
    ]
    for path in candidates:
        if not path:
            continue
        if os.path.isdir(path):
            if path in sys.path:
                sys.path.remove(path)
            sys.path.insert(0, path)


_prepend_dftp_arm_candidates()

try:
    from RH56DFTP.RH56DFTP_TCP import RH56DFTP_TCP
except Exception as e:
    raise RuntimeError(
        "Cannot import RH56DFTP. Run in the same environment where "
        "`grasp_object.py` works (e.g. conda env `rh56`) and/or set "
        "`DFTP_ARM_PATH=/home/unitree/DFTP_arm`."
    ) from e


DEG2RAD = math.pi / 180.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class G1JointIndex:
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

    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14

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

    kNotUsedJoint = 29


WAIST_JOINTS = (G1JointIndex.WaistYaw, G1JointIndex.WaistRoll, G1JointIndex.WaistPitch)

JOINT_NAME_TO_INDEX = {
    "left_shoulder_pitch_joint": G1JointIndex.LeftShoulderPitch,
    "left_shoulder_roll_joint": G1JointIndex.LeftShoulderRoll,
    "left_shoulder_yaw_joint": G1JointIndex.LeftShoulderYaw,
    "left_elbow_joint": G1JointIndex.LeftElbow,
    "left_wrist_roll_joint": G1JointIndex.LeftWristRoll,
    "left_wrist_pitch_joint": G1JointIndex.LeftWristPitch,
    "left_wrist_yaw_joint": G1JointIndex.LeftWristYaw,
    "right_shoulder_pitch_joint": G1JointIndex.RightShoulderPitch,
    "right_shoulder_roll_joint": G1JointIndex.RightShoulderRoll,
    "right_shoulder_yaw_joint": G1JointIndex.RightShoulderYaw,
    "right_elbow_joint": G1JointIndex.RightElbow,
    "right_wrist_roll_joint": G1JointIndex.RightWristRoll,
    "right_wrist_pitch_joint": G1JointIndex.RightWristPitch,
    "right_wrist_yaw_joint": G1JointIndex.RightWristYaw,
    "waist_yaw_joint": G1JointIndex.WaistYaw,
    "waist_roll_joint": G1JointIndex.WaistRoll,
    "waist_pitch_joint": G1JointIndex.WaistPitch,
}

GRASP_PRESETS = {
    "power": {
        "close": [1800, 1800, 1800, 1800, 1800, 0],
        "force": [250, 300, 300, 350, 400, 180],
        "speed": [120, 120, 120, 120, 120, 80],
    },
    "ok_pinch": {
        "close": [0, 0, 0, 1200, 500, 2000],
        "force": [0, 0, 0, 180, 220, 0],
        "speed": [0, 0, 0, 45, 35, 15],
    },
}


@dataclass
class PhaseState:
    phase: str = "traj"  # traj -> approach -> hold -> blend -> done
    t: float = 0.0
    approach_offset_rad: float = 0.0
    contact_reached: bool = False


class ContactApproachController:
    def __init__(self, args):
        self.args = args
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state: Optional[LowState_] = None
        self.first_lowstate = False
        self.crc = CRC()

        self.arm_sdk_publisher = None
        self.lowstate_subscriber = None
        self.ctrl_thread = None

        self.control_dt = 0.02
        self.kp = float(args.kp)
        self.kd = float(args.kd)
        self.kp_waist = float(args.kp_waist)
        self.kd_waist = float(args.kd_waist)

        self.enable_wrist_py = bool(args.enable_wrist_py)

        self.arm_joints = [
            G1JointIndex.LeftShoulderPitch,
            G1JointIndex.LeftShoulderRoll,
            G1JointIndex.LeftShoulderYaw,
            G1JointIndex.LeftElbow,
            G1JointIndex.LeftWristRoll,
            G1JointIndex.RightShoulderPitch,
            G1JointIndex.RightShoulderRoll,
            G1JointIndex.RightShoulderYaw,
            G1JointIndex.RightElbow,
            G1JointIndex.RightWristRoll,
        ]
        if self.enable_wrist_py:
            self.arm_joints += [
                G1JointIndex.LeftWristPitch,
                G1JointIndex.LeftWristYaw,
                G1JointIndex.RightWristPitch,
                G1JointIndex.RightWristYaw,
            ]

        self.joint_to_pose_i = {j: i for i, j in enumerate(self.arm_joints)}
        self.waist_to_pose_i = {j: i for i, j in enumerate(WAIST_JOINTS)}

        self.q_start_arms = None
        self.q_start_waist = None
        self.waist_hold_q = None

        self.keyframes = []
        self.durations = []
        self.poses = None
        self.waist_poses = None
        self.first_empty_seg = None
        self.empty_segments = set()
        self.skip_empty_requested = False
        self.approach_consumed = False

        self.state = PhaseState()
        self.approach_joint_idx = JOINT_NAME_TO_INDEX[args.approach_joint]
        if self.approach_joint_idx not in self.arm_joints:
            raise ValueError("Approach joint must be in controlled arm joints; use --enable-wrist-py if needed")
        self.approach_pose_i = self.joint_to_pose_i[self.approach_joint_idx]

        self.approach_dir = 1.0 if args.approach_dir == "+" else -1.0
        self.approach_step = float(args.approach_step_deg) * DEG2RAD
        self.approach_max = abs(float(args.approach_max_deg) * DEG2RAD)
        self.approach_period = max(0.02, float(args.approach_period))
        self.last_approach_cmd_t = 0.0
        self.last_approach_log_t = 0.0
        self.approach_start_t = 0.0
        self.approach_time_limit_s = None
        self.approach_base_pose = None
        self.approach_seg_idx = None
        self.approach_seg_cum = 0.0
        self.left_wrist_yaw_min_rad = float(args.left_wrist_yaw_min_deg) * DEG2RAD
        self.contact_hold_pose = None
        self.hold_start_t = 0.0
        self.hold_until_t = None
        self.resume_target_t = 0.0
        self.resume_blend_start_t = 0.0
        self.resume_blend_q0 = None
        self.resume_blend_q1 = None

        self.hand = RH56DFTP_TCP(host=args.hand_ip, port=args.hand_port)
        self.contact_channels = [int(x) for x in args.contact_channels.split(",") if x.strip()]
        self.force_baseline = None
        self.last_force_poll_t = 0.0
        self.contact_hit_streak = 0
        self.baseline_samples = []
        self.baseline_ready = False
        self.last_contact_diag_t = 0.0

        hand_log_level = getattr(logging, str(args.hand_log_level).upper(), logging.WARNING)
        logging.getLogger("RH56DFTP").setLevel(hand_log_level)
        logging.getLogger("pymodbus").setLevel(hand_log_level)

        self.done = False

    def load_dataset(self):
        import json

        with open(self.args.dataset_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if not isinstance(data, list):
            raise ValueError("Dataset must be a list")

        self.keyframes = []
        for step in data:
            frame = step.get("frame", []) if isinstance(step, dict) else []
            duration = float(step.get("duration", 0.6)) if isinstance(step, dict) else 0.6
            duration = min(float(self.args.dataset_duration_max), max(0.05, duration))
            self.keyframes.append({"frame": frame if isinstance(frame, list) else [], "duration": duration})

        if not self.keyframes:
            raise ValueError("Empty dataset")

        self.durations = [kf["duration"] for kf in self.keyframes]
        empties = [i for i, kf in enumerate(self.keyframes) if len(kf["frame"]) == 0]
        self.empty_segments = set(empties)
        self.first_empty_seg = empties[0] if empties else None
        self.total_time = sum(self.durations)

    def init_channels(self):
        self.arm_sdk_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_sdk_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.on_lowstate, 10)

    def on_lowstate(self, msg: LowState_):
        self.low_state = msg
        if not self.first_lowstate:
            self.first_lowstate = True

        if self.waist_hold_q is None:
            self.waist_hold_q = {
                G1JointIndex.WaistYaw: msg.motor_state[G1JointIndex.WaistYaw].q,
                G1JointIndex.WaistRoll: msg.motor_state[G1JointIndex.WaistRoll].q,
                G1JointIndex.WaistPitch: msg.motor_state[G1JointIndex.WaistPitch].q,
            }

        if self.q_start_arms is None:
            self.q_start_arms = [msg.motor_state[j].q for j in self.arm_joints]
            self.q_start_waist = [msg.motor_state[j].q for j in WAIST_JOINTS]
            self.build_poses()

    def build_poses(self):
        pose_prev = list(self.q_start_arms)
        waist_prev = list(self.q_start_waist)
        poses = [pose_prev]
        waist_poses = [waist_prev]

        for kf in self.keyframes:
            pose_next = pose_prev[:]
            waist_next = waist_prev[:]
            for item in kf["frame"]:
                if not isinstance(item, dict):
                    continue
                name = item.get("name")
                if name not in JOINT_NAME_TO_INDEX:
                    continue
                idx = JOINT_NAME_TO_INDEX[name]
                ang = float(item.get("angle", 0.0)) * DEG2RAD
                if idx in WAIST_JOINTS:
                    # Keep waist locked in this workflow.
                    continue
                pi = self.joint_to_pose_i.get(idx)
                if pi is not None:
                    pose_next[pi] = ang
            poses.append(pose_next)
            waist_poses.append(waist_next)
            pose_prev, waist_prev = pose_next, waist_next

        self.poses = poses
        self.waist_poses = waist_poses

    def start(self):
        self.ctrl_thread = RecurrentThread(interval=self.control_dt, target=self.control_step, name="contact_approach")
        self.ctrl_thread.Start()

    def _segment_index_and_cum(self, t):
        cum = 0.0
        for i, d in enumerate(self.durations):
            if cum + d > t:
                return i, cum
            cum += d
        return len(self.durations) - 1, cum - self.durations[-1]

    def read_forces(self):
        return [int(self.hand.get(f"FORCE_ACT({i})")) for i in range(6)]

    def request_skip_empty(self):
        self.skip_empty_requested = True

    def _consume_skip_on_empty(self, seg, cum):
        if not self.skip_empty_requested:
            return False
        if seg not in self.empty_segments:
            return False
        self.skip_empty_requested = False
        seg_dur = max(0.05, float(self.durations[seg]))
        self.state.t = cum + seg_dur + 1e-4
        print(f"[INFO] Skipped empty segment #{seg}")
        return True

    def _update_force_baseline(self, forces):
        self.baseline_samples.append(list(forces))
        window = max(1, int(self.args.contact_baseline_samples))
        if len(self.baseline_samples) > window:
            self.baseline_samples.pop(0)
        if len(self.baseline_samples) >= window and not self.baseline_ready:
            cols = list(zip(*self.baseline_samples))
            self.force_baseline = [sum(col) / float(len(col)) for col in cols]
            self.baseline_ready = True
            print(f"[INFO] Contact baseline ready: {[round(v, 1) for v in self.force_baseline]}")

    def contact_reached(self):
        now = time.monotonic()
        if now - self.last_force_poll_t < max(0.02, self.args.contact_poll_s):
            return False
        self.last_force_poll_t = now

        forces = self.read_forces()
        if not self.baseline_ready:
            self._update_force_baseline(forces)
            return False

        deltas = [abs(forces[i] - self.force_baseline[i]) for i in range(6)]
        touched_now = any(
            deltas[i] >= self.args.contact_delta_threshold for i in self.contact_channels
        )
        if touched_now:
            self.contact_hit_streak += 1
        else:
            self.contact_hit_streak = 0

        touched = self.contact_hit_streak >= max(1, int(self.args.contact_consecutive))
        if now - self.last_contact_diag_t >= max(0.2, float(self.args.contact_diag_interval)):
            self.last_contact_diag_t = now
            ch_dbg = [(i, round(deltas[i], 1)) for i in self.contact_channels]
            print(f"[DBG] contact streak={self.contact_hit_streak} deltas={ch_dbg}")
        if touched:
            print(f"[INFO] Contact reached. force={forces}, delta={deltas}")
            self.contact_hit_streak = 0
        return touched

    def _enter_hold(self, q_hold, reason):
        self.contact_hold_pose = list(q_hold)
        self.state.phase = "hold"
        self.hold_start_t = time.monotonic()
        self.hold_until_t = None
        if self.approach_time_limit_s is not None and self.approach_start_t > 0.0:
            # End of current empty-frame window; used for auto-continue.
            self.hold_until_t = self.approach_start_t + self.approach_time_limit_s
        print(f"[INFO] HOLD: {reason}")

    def apply_waist_hold(self):
        q_cmd = [self.waist_hold_q[j] for j in WAIST_JOINTS]
        for j in WAIST_JOINTS:
            wi = self.waist_to_pose_i[j]
            self.low_cmd.motor_cmd[j].tau = 0.0
            self.low_cmd.motor_cmd[j].dq = 0.0
            self.low_cmd.motor_cmd[j].q = float(q_cmd[wi])
            self.low_cmd.motor_cmd[j].kp = self.kp_waist
            self.low_cmd.motor_cmd[j].kd = self.kd_waist

    def write_arm_pose(self, q_arm):
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0
        for i, j in enumerate(self.arm_joints):
            self.low_cmd.motor_cmd[j].tau = 0.0
            self.low_cmd.motor_cmd[j].dq = 0.0
            self.low_cmd.motor_cmd[j].q = float(q_arm[i])
            self.low_cmd.motor_cmd[j].kp = self.kp
            self.low_cmd.motor_cmd[j].kd = self.kd
        self.apply_waist_hold()
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.arm_sdk_publisher.Write(self.low_cmd)

    def _sample_traj_pose_at_t(self, t_val):
        seg, cum = self._segment_index_and_cum(t_val)
        seg_t = t_val - cum
        seg_d = max(0.05, float(self.durations[seg]))
        r = clamp(seg_t / seg_d, 0.0, 1.0)
        r = r * r * (3.0 - 2.0 * r)
        q0 = self.poses[seg]
        q1 = self.poses[seg + 1]
        return [(1.0 - r) * q0[i] + r * q1[i] for i in range(len(q0))]

    def _start_resume_blend(self, target_t):
        self.resume_target_t = float(target_t)
        self.resume_blend_start_t = time.monotonic()
        self.resume_blend_q0 = (
            list(self.contact_hold_pose) if self.contact_hold_pose is not None else list(self.approach_base_pose)
        )
        self.resume_blend_q1 = self._sample_traj_pose_at_t(self.resume_target_t)
        self.state.phase = "blend"
        print("[INFO] Smooth resume to JSON trajectory.")

    def control_step(self):
        if self.done or self.low_state is None or self.poses is None:
            return

        self.state.t += self.control_dt

        if self.state.phase == "traj":
            if self.state.t >= self.total_time:
                self.done = True
                return
            seg, cum = self._segment_index_and_cum(self.state.t)
            if self._consume_skip_on_empty(seg, cum):
                return
            if (not self.approach_consumed) and self.first_empty_seg is not None and seg >= self.first_empty_seg:
                # Enter contact approach from current pose.
                self.approach_consumed = True
                self.state.phase = "approach"
                self.approach_seg_idx = seg
                self.approach_seg_cum = cum
                self.approach_base_pose = list(self.poses[seg])
                self.approach_start_t = time.monotonic()
                seg_d = max(0.05, float(self.durations[seg]))
                seg_t = max(0.0, self.state.t - cum)
                remaining_in_empty = max(0.05, seg_d - seg_t)
                if float(self.args.approach_max_sec) > 0.0:
                    self.approach_time_limit_s = float(self.args.approach_max_sec)
                else:
                    self.approach_time_limit_s = remaining_in_empty
                self.force_baseline = None
                self.baseline_samples = []
                self.baseline_ready = False
                self.contact_hit_streak = 0
                print(
                    f"[INFO] Enter approach phase at seg={seg}, collecting force baseline... "
                    f"time_limit={self.approach_time_limit_s:.2f}s"
                )
                self.write_arm_pose(self.approach_base_pose)
                return

            seg_t = self.state.t - cum
            seg_d = max(0.05, float(self.durations[seg]))
            r = clamp(seg_t / seg_d, 0.0, 1.0)
            r = r * r * (3.0 - 2.0 * r)
            q0 = self.poses[seg]
            q1 = self.poses[seg + 1]
            q = [(1.0 - r) * q0[i] + r * q1[i] for i in range(len(q0))]
            self.write_arm_pose(q)
            return

        if self.state.phase == "approach":
            now = time.monotonic()
            if self.approach_time_limit_s is not None:
                if (now - self.approach_start_t) >= self.approach_time_limit_s:
                    q_limit = list(self.approach_base_pose)
                    q_limit[self.approach_pose_i] = q_limit[self.approach_pose_i] + self.state.approach_offset_rad
                    self._enter_hold(q_limit, f"approach time limit reached ({self.approach_time_limit_s:.2f}s)")
                    return
            if now - self.last_approach_cmd_t >= self.approach_period:
                self.last_approach_cmd_t = now
                self.state.approach_offset_rad += self.approach_dir * self.approach_step
                if abs(self.state.approach_offset_rad) > self.approach_max:
                    self.state.approach_offset_rad = math.copysign(self.approach_max, self.state.approach_offset_rad)

                q = list(self.approach_base_pose)
                q[self.approach_pose_i] = q[self.approach_pose_i] + self.state.approach_offset_rad

                # Safety limit: do not move left_wrist_yaw below configured minimum.
                if self.approach_joint_idx == G1JointIndex.LeftWristYaw:
                    if q[self.approach_pose_i] <= self.left_wrist_yaw_min_rad:
                        q[self.approach_pose_i] = self.left_wrist_yaw_min_rad
                        self.write_arm_pose(q)
                        self._enter_hold(
                            q,
                            f"left_wrist_yaw limit reached at {self.args.left_wrist_yaw_min_deg:.1f} deg",
                        )
                        return

                self.write_arm_pose(q)
                if now - self.last_approach_log_t >= max(0.1, float(self.args.approach_log_interval)):
                    self.last_approach_log_t = now
                    print(
                        f"[INFO] approach offset_deg={self.state.approach_offset_rad / DEG2RAD:.2f}, "
                        f"joint={self.args.approach_joint}"
                    )

            if self.contact_reached():
                self.state.contact_reached = True
                q_contact = list(self.approach_base_pose)
                q_contact[self.approach_pose_i] = q_contact[self.approach_pose_i] + self.state.approach_offset_rad
                self._enter_hold(q_contact, "contact detected")
                if self.args.grasp_on_contact:
                    self.run_grasp()
                return

            if abs(self.state.approach_offset_rad) >= self.approach_max:
                q_limit = list(self.approach_base_pose)
                q_limit[self.approach_pose_i] = q_limit[self.approach_pose_i] + self.state.approach_offset_rad
                self._enter_hold(q_limit, "approach max reached before contact")
                return

        if self.state.phase == "hold":
            if self.contact_hold_pose is not None:
                self.write_arm_pose(self.contact_hold_pose)
            now = time.monotonic()
            continue_by_enter = self.skip_empty_requested
            if continue_by_enter:
                self.skip_empty_requested = False
                print("[INFO] Resume JSON by Enter.")

            continue_by_empty_timeout = self.hold_until_t is not None and now >= self.hold_until_t
            hold_sec = float(self.args.hold_after_contact_sec)
            continue_by_hold_sec = hold_sec > 0.0 and (now - self.hold_start_t) >= hold_sec

            if continue_by_enter or continue_by_empty_timeout or continue_by_hold_sec:
                if self.args.resume_json_after_hold and self.approach_seg_idx is not None:
                    seg_dur = max(0.05, float(self.durations[self.approach_seg_idx]))
                    t_resume = self.approach_seg_cum + seg_dur + 1e-4
                    self._start_resume_blend(t_resume)
                    self.contact_hold_pose = None
                    self.hold_until_t = None
                    print("[INFO] Resume JSON after hold (with blend).")
                else:
                    self.state.phase = "done"
                    self.done = True

        if self.state.phase == "blend":
            # Freeze logical timeline while blending back into JSON.
            self.state.t = self.resume_target_t
            q0 = self.resume_blend_q0
            q1 = self.resume_blend_q1
            if q0 is None or q1 is None:
                self.state.phase = "traj"
                return
            blend_d = max(0.05, float(self.args.resume_blend_sec))
            rb = clamp((time.monotonic() - self.resume_blend_start_t) / blend_d, 0.0, 1.0)
            rb = rb * rb * (3.0 - 2.0 * rb)
            q = [(1.0 - rb) * q0[i] + rb * q1[i] for i in range(len(q0))]
            self.write_arm_pose(q)
            if rb >= 1.0:
                self.state.phase = "traj"
                print("[INFO] Blend complete. JSON timing restored.")

    def run_grasp(self):
        preset = GRASP_PRESETS[self.args.grasp_preset]
        close_targets = list(preset["close"])
        force_limits = list(preset["force"])
        speed_limits = list(preset["speed"])

        def set_positions(vals):
            for i, v in enumerate(vals):
                self.hand.set(f"POS_SET({i})", int(v))

        def set_speeds(vals):
            for i, v in enumerate(vals):
                self.hand.set(f"SPEED_SET({i})", int(v))

        def set_forces(vals):
            for i, v in enumerate(vals):
                self.hand.set(f"FORCE_SET({i})", int(v))

        def read_forces_local():
            return [int(self.hand.get(f"FORCE_ACT({i})")) for i in range(6)]

        print(f"[INFO] Grasp start. preset={self.args.grasp_preset}")
        current_pos = [self.args.hand_open_pos] * 6
        frozen = [False] * 6
        moved_once = [False] * 6

        set_positions(current_pos)
        time.sleep(0.4)
        set_speeds(speed_limits)
        time.sleep(0.1)
        set_forces(force_limits)
        time.sleep(0.1)
        baseline = read_forces_local()

        while True:
            done = True
            forces = read_forces_local()
            delta = [max(0, forces[i] - baseline[i]) for i in range(6)]
            for i in range(6):
                if frozen[i]:
                    continue
                if moved_once[i] and delta[i] >= max(0, force_limits[i] - self.args.grasp_margin):
                    frozen[i] = True
                    continue
                if current_pos[i] < close_targets[i]:
                    current_pos[i] = min(close_targets[i], current_pos[i] + self.args.grasp_step)
                    moved_once[i] = True
                    done = False
            set_positions(current_pos)
            if all(frozen) or done:
                break
            time.sleep(self.args.grasp_sleep)

        print(f"[INFO] Grasp done. pos={current_pos}")
        if self.args.release_after > 0:
            time.sleep(self.args.release_after)
            set_positions([self.args.hand_open_pos] * 6)
            print("[INFO] Hand released")

    def shutdown(self):
        try:
            if self.args.release_arm_sdk_on_exit:
                # Optional release of arm_sdk ownership on exit.
                self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 0.0
                self.low_cmd.crc = self.crc.Crc(self.low_cmd)
                self.arm_sdk_publisher.Write(self.low_cmd)
        except Exception:
            pass
        if self.lowstate_subscriber is not None:
            self.lowstate_subscriber.Close()
        try:
            self.hand.close()
        except Exception:
            pass


def main():
    p = argparse.ArgumentParser(description="Move arm to pre-grasp pose, approach until palm contact, then optionally grasp.")
    p.add_argument("iface", nargs="?", default=None)
    p.add_argument("--dataset-path", required=True)
    p.add_argument("--dataset-duration-max", type=float, default=120.0)
    p.add_argument("--enable-wrist-py", action="store_true")

    p.add_argument("--kp", type=float, default=60.0)
    p.add_argument("--kd", type=float, default=1.5)
    p.add_argument("--kp-waist", type=float, default=250.0)
    p.add_argument("--kd-waist", type=float, default=6.0)

    p.add_argument("--approach-joint", default="left_wrist_yaw_joint", choices=sorted(JOINT_NAME_TO_INDEX.keys()))
    p.add_argument("--approach-dir", choices=["+", "-"], default="+")
    p.add_argument("--approach-step-deg", type=float, default=0.3)
    p.add_argument("--approach-max-deg", type=float, default=30.0)
    p.add_argument("--approach-period", type=float, default=0.2)
    p.add_argument(
        "--approach-max-sec",
        type=float,
        default=0.0,
        help="If >0, use this approach duration limit. If <=0, use remaining empty-frame duration.",
    )
    p.add_argument("--approach-log-interval", type=float, default=0.5)
    p.add_argument("--resume-blend-sec", type=float, default=0.7)
    p.add_argument(
        "--left-wrist-yaw-min-deg",
        type=float,
        default=-30.0,
        help="Absolute minimum angle for left_wrist_yaw during approach.",
    )

    p.add_argument("--hand-ip", default="192.168.123.210")
    p.add_argument("--hand-port", type=int, default=6000)
    p.add_argument("--contact-delta-threshold", type=float, default=35.0)
    p.add_argument("--contact-consecutive", type=int, default=3)
    p.add_argument("--contact-baseline-samples", type=int, default=8)
    p.add_argument("--contact-channels", default="0,1,2,3,4,5")
    p.add_argument("--contact-poll-s", type=float, default=0.12)
    p.add_argument("--contact-diag-interval", type=float, default=0.5)
    p.add_argument("--hold-after-contact-sec", type=float, default=-1.0)
    p.add_argument("--startup-timeout-s", type=float, default=8.0)
    p.add_argument("--release-arm-sdk-on-exit", dest="release_arm_sdk_on_exit", action="store_true")
    p.add_argument("--no-release-arm-sdk-on-exit", dest="release_arm_sdk_on_exit", action="store_false")
    p.add_argument(
        "--resume-json-after-hold",
        dest="resume_json_after_hold",
        action="store_true",
        help="After hold on contact, continue remaining JSON frames (after empty segment).",
    )
    p.add_argument(
        "--no-resume-json-after-hold",
        dest="resume_json_after_hold",
        action="store_false",
        help="Stop after hold; do not continue remaining JSON.",
    )
    p.add_argument(
        "--hand-log-level",
        default="WARNING",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
        help="Log level for RH56DFTP/pymodbus internals",
    )

    p.add_argument("--grasp-on-contact", action="store_true")
    p.add_argument("--grasp-preset", choices=sorted(GRASP_PRESETS.keys()), default="power")
    p.add_argument("--hand-open-pos", type=int, default=0)
    p.add_argument("--grasp-step", type=int, default=120)
    p.add_argument("--grasp-sleep", type=float, default=0.08)
    p.add_argument("--grasp-margin", type=int, default=20)
    p.add_argument("--release-after", type=float, default=0.0)
    p.add_argument(
        "--no-skip-empty-on-enter",
        dest="skip_empty_on_enter",
        action="store_false",
        help="Disable Enter-to-skip for empty frame segments.",
    )
    p.set_defaults(skip_empty_on_enter=True, release_arm_sdk_on_exit=True, resume_json_after_hold=True)

    args = p.parse_args()

    print("WARNING: ensure free space around robot and hand.")
    input("Press Enter to continue... ")

    if args.iface is not None:
        ChannelFactoryInitialize(0, args.iface)
    else:
        ChannelFactoryInitialize(0)

    ctrl = ContactApproachController(args)
    ctrl.load_dataset()
    ctrl.init_channels()

    t0 = time.monotonic()
    while not ctrl.first_lowstate:
        if (time.monotonic() - t0) >= float(args.startup_timeout_s):
            raise RuntimeError(
                f"No lowstate within {args.startup_timeout_s:.1f}s. "
                "Check robot state/network/topic."
            )
        time.sleep(0.1)

    print(f"[INFO] lowstate received, dataset loaded: {args.dataset_path}")
    print(f"[INFO] first empty frame segment: {ctrl.first_empty_seg}")
    print(f"[INFO] approach joint={args.approach_joint}, dir={args.approach_dir}")
    if ctrl.first_empty_seg is None:
        print("[WARN] No empty frame in dataset: contact approach phase will never start.")
    if args.skip_empty_on_enter:
        print("[INFO] Skip-empty mode ON: press Enter to skip empty segment.")

    ctrl.start()

    try:
        while not ctrl.done:
            if args.skip_empty_on_enter:
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if ready:
                    _ = sys.stdin.readline()
                    ctrl.request_skip_empty()
            else:
                time.sleep(0.2)
    finally:
        ctrl.shutdown()
        print("[INFO] Finished")


if __name__ == "__main__":
    main()

