#!/usr/bin/env python3
from __future__ import annotations

"""Add one compensating extension frame to IK JSON.

Adds a final frame with:
- elbow angle increased by +5 deg
- shoulder pitch decreased by -5 deg

python3 4step_add_push_frame.py

Defaults can be overridden via CLI flags.
"""

import argparse
import json
import os
import sys
from typing import Any


def find_last_angle(frames: list[dict[str, Any]], joint_name: str) -> float | None:
    for step in reversed(frames):
        frame = step.get("frame", [])
        if not isinstance(frame, list):
            continue
        for item in frame:
            if isinstance(item, dict) and item.get("name") == joint_name:
                try:
                    return float(item.get("angle"))
                except (TypeError, ValueError):
                    return None
    return None


def set_or_add_joint(frame_items: list[dict[str, Any]], joint_name: str, angle: float) -> None:
    for item in frame_items:
        if isinstance(item, dict) and item.get("name") == joint_name:
            item["angle"] = float(angle)
            return
    frame_items.append({"name": joint_name, "angle": float(angle)})


def main() -> int:
    parser = argparse.ArgumentParser(description="Append one extension frame to dataset JSON")
    parser.add_argument(
        "input_json",
        nargs="?",
        default=os.path.join(os.path.dirname(__file__), "./pipeline/ik_records/current_target_right.json"),
        help="Input JSON path (default: 216.json used in 3step)",
    )
    parser.add_argument("--output", default=None, help="Output JSON path (default: overwrite input)")
    parser.add_argument("--side", choices=["right", "left"], default="right")
    parser.add_argument("--duration", type=float, default=0.5)
    parser.add_argument("--elbow-delta-deg", type=float, default=10.0)
    parser.add_argument("--pitch-delta-deg", type=float, default=-10.0)
    args = parser.parse_args()

    if not os.path.isfile(args.input_json):
        print(f"Input file not found: {args.input_json}", file=sys.stderr)
        return 2

    with open(args.input_json, "r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, list) or not data:
        print("Input JSON must be non-empty list of {frame, duration}", file=sys.stderr)
        return 2

    last = data[-1]
    if not isinstance(last, dict) or "frame" not in last or not isinstance(last["frame"], list):
        print("Last item has invalid 'frame' format", file=sys.stderr)
        return 2

    prefix = "right" if args.side == "right" else "left"
    elbow_joint = f"{prefix}_elbow_joint"
    shoulder_pitch_joint = f"{prefix}_shoulder_pitch_joint"

    base_elbow = find_last_angle(data, elbow_joint)
    base_pitch = find_last_angle(data, shoulder_pitch_joint)
    if base_elbow is None:
        print(f"Joint not found in dataset: {elbow_joint}", file=sys.stderr)
        return 2
    if base_pitch is None:
        print(f"Joint not found in dataset: {shoulder_pitch_joint}", file=sys.stderr)
        return 2

    new_frame_items: list[dict[str, Any]] = []
    for item in last["frame"]:
        if isinstance(item, dict) and "name" in item:
            copied = dict(item)
            if "angle" in copied:
                try:
                    copied["angle"] = float(copied["angle"])
                except (TypeError, ValueError):
                    pass
            new_frame_items.append(copied)

    set_or_add_joint(new_frame_items, elbow_joint, base_elbow + float(args.elbow_delta_deg))
    set_or_add_joint(new_frame_items, shoulder_pitch_joint, base_pitch + float(args.pitch_delta_deg))

    new_step = {
        "frame": new_frame_items,
        "duration": float(args.duration),
    }
    data.append(new_step)

    out_path = args.output or args.input_json
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

    print(f"Appended 1 frame to: {out_path}")
    print(
        f"{elbow_joint}: {base_elbow:.3f} -> {base_elbow + float(args.elbow_delta_deg):.3f} deg, "
        f"{shoulder_pitch_joint}: {base_pitch:.3f} -> {base_pitch + float(args.pitch_delta_deg):.3f} deg"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

