#!/usr/bin/env python3
"""Проверка CSV телеметрии WearBench-G1 без SDK и робота"""

import argparse
import csv
import json
import math
import statistics
from pathlib import Path

from record_lowstate import JOINT_NAMES, columns


def analyze(csv_path):
    with csv_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames != columns():
            raise ValueError("CSV schema mismatch")
        prev_time = None
        prev_seq = None
        times = []
        skipped_sequences = 0
        rows = 0
        nonfinite = 0
        for record in reader:
            rows += 1
            t = int(record["monotonic_ns"])
            seq = int(record["sequence"])
            if prev_time is not None:
                if t <= prev_time:
                    raise ValueError("Non-increasing timestamps at row " + str(rows))
                if seq <= prev_seq:
                    raise ValueError("Non-increasing sequence at row " + str(rows))
                skipped_sequences += seq - prev_seq - 1
                times.append((t - prev_time) / 1e9)
            prev_time = t
            prev_seq = seq
            for joint in JOINT_NAMES:
                for suffix in ("q", "dq", "tau_est"):
                    if not math.isfinite(float(record[joint + "_" + suffix])):
                        nonfinite += 1
        if rows < 2:
            raise ValueError("Not enough samples (need >=2)")
    duration = sum(times)
    meta_path = csv_path.with_suffix(".meta.json")
    meta = json.loads(meta_path.read_text()) if meta_path.exists() else {}
    summary = {
        "file": str(csv_path),
        "samples": rows,
        "duration_s": round(duration, 3),
        "observed_rate_hz": round((rows - 1) / duration, 2),
        "median_interval_ms": round(1000 * statistics.median(times), 3),
        "max_interval_ms": round(1000 * max(times), 3),
        "sequence_gaps": skipped_sequences,
        "nonfinite_q_dq_tau": nonfinite,
        "recorded_drops": meta.get("queue_dropped"),
        "recorded_decode_errors": meta.get("decode_errors"),
        "reason": meta.get("reason"),
        "usable_for_nominal_analysis": bool(
            rows >= 10 and skipped_sequences == 0 and nonfinite == 0
            and meta.get("queue_dropped", 0) == 0
            and meta.get("decode_errors", 0) == 0
            and meta.get("reason") in ("duration", "interrupted")
        ),
    }
    return summary


def main():
    p = argparse.ArgumentParser()
    p.add_argument("csv_path", type=Path)
    args = p.parse_args()
    try:
        summary = analyze(args.csv_path)
    except Exception as exc:
        print("INVALID:", repr(exc))
        return 2
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if summary["usable_for_nominal_analysis"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
