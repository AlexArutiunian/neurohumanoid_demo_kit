#!/usr/bin/env python3
import argparse
import json
import math
import threading
import time
from dataclasses import dataclass, asdict
from datetime import datetime
from typing import Dict, List, Optional

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_


@dataclass
class StateSnapshot:
    timestamp: float
    x: float
    y: float
    yaw: float
    vx: float
    vy: float


@dataclass
class TrialResult:
    speed_mps: float
    reached_target: bool
    elapsed_s: float
    progress_m: float
    final_lateral_error_m: float
    max_abs_lateral_error_m: float
    telemetry: List[Dict[str, float]]


class SportStateTracker:
    def __init__(self, topic: str):
        self._topic = topic
        self._sub = ChannelSubscriber(topic, SportModeState_)
        self._lock = threading.Lock()
        self._latest: Optional[StateSnapshot] = None

    def start(self):
        self._sub.Init(self._on_state, 50)

    def close(self):
        self._sub.Close()

    def _on_state(self, msg: SportModeState_):
        snapshot = StateSnapshot(
            timestamp=time.time(),
            x=float(msg.position[0]),
            y=float(msg.position[1]),
            yaw=float(msg.imu_state.rpy[2]),
            vx=float(msg.velocity[0]),
            vy=float(msg.velocity[1]),
        )
        with self._lock:
            self._latest = snapshot

    def latest(self) -> Optional[StateSnapshot]:
        with self._lock:
            return self._latest

    def wait_first(self, timeout_s: float) -> Optional[StateSnapshot]:
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            s = self.latest()
            if s is not None:
                return s
            time.sleep(0.02)
        return None


def parse_speeds(raw: str) -> List[float]:
    speeds = []
    for token in raw.split(","):
        token = token.strip()
        if not token:
            continue
        v = float(token)
        if v <= 0:
            raise ValueError(f"Speed must be > 0, got {v}")
        speeds.append(v)
    if not speeds:
        raise ValueError("No speeds provided")
    return speeds


def run_trial(
    loco: LocoClient,
    tracker: SportStateTracker,
    speed_mps: float,
    cmd_vx: float,
    cmd_vy: float,
    cmd_omega: float,
    distance_m: float,
    max_time_s: float,
    command_refresh_s: float,
    telemetry_dt_s: float,
) -> TrialResult:
    start = tracker.wait_first(2.0)
    if start is None:
        raise RuntimeError("No sport mode state before trial")

    yaw0 = start.yaw
    cmd_norm = math.hypot(cmd_vx, cmd_vy)
    if cmd_norm <= 1e-6:
        raise ValueError("Command vector is zero; choose non-zero walking direction.")
    # Convert commanded body-frame direction into a world-frame unit direction.
    bx = cmd_vx / cmd_norm
    by = cmd_vy / cmd_norm
    ux = math.cos(yaw0) * bx - math.sin(yaw0) * by
    uy = math.sin(yaw0) * bx + math.cos(yaw0) * by

    x0 = start.x
    y0 = start.y

    reached_target = False
    max_abs_lat = 0.0
    last_cmd_t = 0.0
    last_telemetry_t = -1e9
    telemetry: List[Dict[str, float]] = []

    t0 = time.time()

    while True:
        now = time.time()
        if now - last_cmd_t >= command_refresh_s:
            loco.Move(cmd_vx, cmd_vy, cmd_omega, continous_move=True)
            last_cmd_t = now

        s = tracker.latest()
        if s is not None:
            dx = s.x - x0
            dy = s.y - y0
            progress = dx * ux + dy * uy
            lateral = -dx * uy + dy * ux
            max_abs_lat = max(max_abs_lat, abs(lateral))

            if now - last_telemetry_t >= telemetry_dt_s:
                speed_actual = math.hypot(s.vx, s.vy)
                telemetry.append(
                    {
                        "t_s": now - t0,
                        "vx_mps": s.vx,
                        "vy_mps": s.vy,
                        "speed_mps": speed_actual,
                        "yaw_rad": s.yaw,
                        "progress_m": progress,
                        "lateral_m": lateral,
                    }
                )
                last_telemetry_t = now

            if progress >= distance_m:
                reached_target = True
                break

        if now - t0 >= max_time_s:
            break

        time.sleep(0.02)

    loco.StopMove()
    time.sleep(0.6)

    end_state = tracker.latest()
    if end_state is None:
        end_state = start

    dx = end_state.x - x0
    dy = end_state.y - y0
    progress = dx * ux + dy * uy
    lateral = -dx * uy + dy * ux

    return TrialResult(
        speed_mps=speed_mps,
        reached_target=reached_target,
        elapsed_s=time.time() - t0,
        progress_m=progress,
        final_lateral_error_m=lateral,
        max_abs_lateral_error_m=max_abs_lat,
        telemetry=telemetry,
    )


def main():
    parser = argparse.ArgumentParser(
        description=(
            "G1 loco test: walk straight for 2m with multiple speeds and measure lateral deviation"
        )
    )
    parser.add_argument("network_interface", nargs="?", default=None)
    parser.add_argument("--state-topic", default="rt/odommodestate")
    parser.add_argument("--distance", type=float, default=0.4)
    parser.add_argument("--speeds", default="0.2")
    parser.add_argument(
        "--direction",
        choices=["forward", "backward", "left", "right"],
        default="forward",
    )
    parser.add_argument("--max-time", type=float, default=5.0)
    parser.add_argument("--command-refresh", type=float, default=0.5)
    parser.add_argument("--telemetry-dt", type=float, default=0.05)
    parser.add_argument("--between-trials", type=float, default=2.0)
    parser.add_argument("--no-prompt", action="store_true")
    parser.add_argument("--json-out", default="")
    args = parser.parse_args()

    speeds = parse_speeds(args.speeds)

    print("WARNING: keep a clear straight area and be ready for emergency stop.")
    print(f"State topic: {args.state_topic}")
    print(f"Distance: {args.distance:.2f} m")
    print(f"Speeds: {', '.join(f'{v:.3f}' for v in speeds)} m/s")
    if not args.no_prompt:
        input("Press Enter to continue...")

    if args.network_interface:
        ChannelFactoryInitialize(0, args.network_interface)
    else:
        ChannelFactoryInitialize(0)

    tracker = SportStateTracker(args.state_topic)
    tracker.start()

    first = tracker.wait_first(5.0)
    if first is None:
        raise RuntimeError(
            f"No data on topic '{args.state_topic}'. Check topic name/network interface."
        )

    loco = LocoClient()
    loco.SetTimeout(10.0)
    loco.Init()

    # Enter locomotion mode.
    loco.Start()
    time.sleep(1.5)

    results: List[TrialResult] = []

    try:
        for i, speed in enumerate(speeds, start=1):
            if not args.no_prompt:
                input(
                    f"Trial {i}/{len(speeds)} at {speed:.3f} m/s. "
                    "Set robot at start line, then press Enter..."
                )

            loco.StopMove()
            time.sleep(0.5)

            if args.direction == "forward":
                cmd_vx, cmd_vy = speed, 0.0
            elif args.direction == "backward":
                cmd_vx, cmd_vy = -speed, 0.0
            elif args.direction == "left":
                cmd_vx, cmd_vy = 0.0, speed
            else:  # right
                cmd_vx, cmd_vy = 0.0, -speed

            result = run_trial(
                loco=loco,
                tracker=tracker,
                speed_mps=speed,
                cmd_vx=cmd_vx,
                cmd_vy=cmd_vy,
                cmd_omega=0.0,
                distance_m=args.distance,
                max_time_s=args.max_time,
                command_refresh_s=args.command_refresh,
                telemetry_dt_s=args.telemetry_dt,
            )
            results.append(result)

            print(
                f"[{i}/{len(speeds)}] speed={result.speed_mps:.3f} m/s, "
                f"reached={result.reached_target}, elapsed={result.elapsed_s:.2f}s, "
                f"progress={result.progress_m:.3f}m, "
                f"lat_final={result.final_lateral_error_m:.3f}m, "
                f"lat_max_abs={result.max_abs_lateral_error_m:.3f}m"
            )

            time.sleep(args.between_trials)

    finally:
        loco.StopMove()
        tracker.close()

    print("\n=== SUMMARY ===")
    for r in results:
        print(
            f"speed={r.speed_mps:.3f} m/s | reached={r.reached_target} | "
            f"time={r.elapsed_s:.2f}s | progress={r.progress_m:.3f}m | "
            f"final_lat={r.final_lateral_error_m:.3f}m | "
            f"max_abs_lat={r.max_abs_lateral_error_m:.3f}m"
        )

    if results:
        slowest = min(results, key=lambda x: x.speed_mps)
        print("\nSmall-speed focus:")
        print(
            f"speed={slowest.speed_mps:.3f} m/s -> "
            f"final lateral error={slowest.final_lateral_error_m:.3f} m, "
            f"max abs lateral error={slowest.max_abs_lateral_error_m:.3f} m"
        )

    out_path = args.json_out.strip()
    if not out_path:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = f"g1_loco_walk_2m_results_{ts}.json"

    payload = {
        "distance_m": args.distance,
        "state_topic": args.state_topic,
        "speeds_mps": speeds,
        "results": [asdict(r) for r in results],
    }

    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)

    print(f"Saved: {out_path}")


if __name__ == "__main__":
    main()

