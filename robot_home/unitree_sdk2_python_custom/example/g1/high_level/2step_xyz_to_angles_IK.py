'''
cd pipeline
uv run deploy/ik_calc_only.py deploy/config/env_FTP.yaml \
  --side right --kind hand --coord local \
  --x 0.35 --y -0.16 --z 0.3 \
  --orient-mode dir --orient-dir 0 0 1 --tip-axis z \
  --orient-weight 0.15 --posture-weight 0.03 --max-arm-delta-deg 110 \
  --ik-attempts 96 --ik-jitter 0.25 \
  --freeze-wrist-yaw \
  --output ik_records/calc_test.json



'''


"""Step 2: read target XYZ from file and run IK.

Reads `dataset_rgbd/current_target.txt` (written by step 1), picks coordinates
for selected side, and runs IK solver with matching `--side`.

Supported target file formats:
1) labeled lines:
   right x y z
   left  x y z
2) unlabeled lines (legacy):
   x y z   # first line -> right
   x y z   # second line -> left
   
python3 2step_xyz_to_angles_IK.py --side right

"""


import argparse
import os
import shlex
import shutil
import subprocess
import sys
import time
from typing import Dict, Optional, Tuple


def _parse_target_file(path: str) -> Dict[str, Tuple[float, float, float]]:
    targets: Dict[str, Tuple[float, float, float]] = {}
    unlabeled = []

    with open(path, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue

            parts = line.replace(",", " ").split()
            if not parts:
                continue

            tag = parts[0].lower()
            if tag in ("right", "left") and len(parts) >= 4:
                try:
                    x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                    targets[tag] = (x, y, z)
                    continue
                except ValueError:
                    pass

            if len(parts) >= 3:
                try:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    unlabeled.append((x, y, z))
                except ValueError:
                    continue

    if "right" not in targets and len(unlabeled) >= 1:
        targets["right"] = unlabeled[0]
    if "left" not in targets and len(unlabeled) >= 2:
        targets["left"] = unlabeled[1]

    return targets


def _split_capture_tag(filename: str) -> Optional[str]:
    suffixes = (
        "_rgb.jpg",
        "_depth_mm.npy",
        "_depth_mm.png",
        "_depth_preview.jpg",
        "_meta.json",
        "_object_crop.jpg",
        "_xyz_cam_m.txt",
        "_xyz_pelvis_m.txt",
    )
    for sfx in suffixes:
        if filename.endswith(sfx):
            return filename[: -len(sfx)]
    return None


def _latest_capture_tag(dataset_dir: str) -> Optional[str]:
    if not os.path.isdir(dataset_dir):
        return None

    latest_mtime = -1.0
    latest_tag = None

    for name in os.listdir(dataset_dir):
        tag = _split_capture_tag(name)
        if not tag:
            continue
        path = os.path.join(dataset_dir, name)
        if not os.path.isfile(path):
            continue
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            continue
        if mtime > latest_mtime:
            latest_mtime = mtime
            latest_tag = tag
    return latest_tag


def _archive_copy(src_json: str, archive_dir: str, side: str, capture_tag: Optional[str]) -> str:
    os.makedirs(archive_dir, exist_ok=True)
    tag = capture_tag or time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
    base_name = f"current_target_{side}_{tag}"
    out_path = os.path.join(archive_dir, f"{base_name}.json")

    if os.path.exists(out_path):
        idx = 1
        while True:
            candidate = os.path.join(archive_dir, f"{base_name}_{idx:02d}.json")
            if not os.path.exists(candidate):
                out_path = candidate
                break
            idx += 1

    shutil.copy2(src_json, out_path)
    return out_path


def main() -> int:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_target = os.path.join(script_dir, "dataset_rgbd", "current_target.txt")
    default_ik_script = os.path.join(script_dir, "pipeline", "deploy", "ik_calc_only.py")
    default_config = os.path.join(script_dir, "pipeline", "deploy", "config", "env_FTP.yaml")
    default_records_dir = os.path.join(script_dir, "pipeline", "ik_records")
    default_dataset_dir = os.path.join(script_dir, "dataset_rgbd")
    default_archive_dir = os.path.join(default_records_dir, "dataset")
    default_uv_project = os.path.join(script_dir, "pipeline")

    parser = argparse.ArgumentParser(description="Read current_target.txt and run IK for selected side")
    parser.add_argument("--side", choices=["right", "left"], required=True)
    parser.add_argument("--target-file", default=default_target)
    parser.add_argument("--ik-script", default=default_ik_script)
    parser.add_argument("--config", default=default_config)
    parser.add_argument("--uv-bin", default="uv")
    parser.add_argument(
        "--runner",
        choices=["auto", "uv", "python"],
        default="auto",
        help="How to run IK script: auto (prefer python from .venv), uv, or python",
    )
    parser.add_argument(
        "--python-bin",
        default=None,
        help="Python interpreter for --runner python (default: pipeline/.venv/bin/python)",
    )
    parser.add_argument("--uv-project", default=default_uv_project)
    parser.add_argument("--dataset-dir", default=default_dataset_dir)
    parser.add_argument("--archive-dir", default=default_archive_dir)
    parser.add_argument("--no-archive", action="store_true", help="Disable non-overwrite archive copy")
    parser.add_argument("--kind", choices=["hand", "index"], default="hand")
    parser.add_argument("--coord", choices=["local", "world"], default="local")

    # IK defaults from your command examples.
    parser.add_argument("--orient-mode", choices=["none", "to_target", "dir"], default="dir")
    parser.add_argument("--orient-dir", nargs=3, type=float, default=[0.0, 0.0, 1.0])
    parser.add_argument("--tip-axis", choices=["x", "y", "z"], default="z")
    parser.add_argument("--orient-weight", type=float, default=0.15)
    parser.add_argument("--posture-weight", type=float, default=0.03)
    parser.add_argument("--max-arm-delta-deg", type=float, default=110.0)
    parser.add_argument(
        "--hand-err-limit",
        type=float,
        default=0.035,
        help="Max IK error for hand in meters (default 0.035 = 3.5 cm)",
    )
    parser.add_argument("--ik-attempts", type=int, default=96)
    parser.add_argument("--ik-jitter", type=float, default=0.25)
    parser.add_argument("--freeze-wrist-yaw", action="store_true", default=True)
    parser.add_argument("--no-freeze-wrist-yaw", dest="freeze_wrist_yaw", action="store_false")

    parser.add_argument(
        "--output",
        default=None,
        help="Output JSON path. Default: pipeline/ik_records/current_target_<side>.json",
    )
    parser.add_argument("--print-only", action="store_true", help="Print IK command without executing")

    args = parser.parse_args()

    if not os.path.isfile(args.target_file):
        print(f"[step2] target file not found: {args.target_file}", file=sys.stderr)
        return 2
    if not os.path.isdir(args.uv_project):
        print(f"[step2] uv project dir not found: {args.uv_project}", file=sys.stderr)
        return 2
    if not os.path.isfile(args.ik_script):
        print(f"[step2] ik script not found: {args.ik_script}", file=sys.stderr)
        return 2
    if not os.path.isfile(args.config):
        print(f"[step2] config not found: {args.config}", file=sys.stderr)
        return 2

    targets = _parse_target_file(args.target_file)
    if args.side not in targets:
        print(
            f"[step2] side '{args.side}' not found in {args.target_file}. "
            f"Available: {sorted(targets.keys())}",
            file=sys.stderr,
        )
        return 2

    x, y, z = targets[args.side]

    if args.output:
        out_path = args.output
    else:
        os.makedirs(default_records_dir, exist_ok=True)
        out_path = os.path.join(default_records_dir, f"current_target_{args.side}.json")

    default_python = os.path.join(args.uv_project, ".venv", "bin", "python")
    python_bin = args.python_bin or default_python

    if args.runner == "auto":
        runner = "python" if os.path.isfile(python_bin) else "uv"
    else:
        runner = args.runner

    base_cmd = [
        args.ik_script,
        args.config,
        "--side",
        args.side,
        "--kind",
        args.kind,
        "--coord",
        args.coord,
        "--x",
        f"{x:.6f}",
        "--y",
        f"{y:.6f}",
        "--z",
        f"{z:.6f}",
        "--orient-mode",
        args.orient_mode,
        "--tip-axis",
        args.tip_axis,
        "--orient-weight",
        str(args.orient_weight),
        "--posture-weight",
        str(args.posture_weight),
        "--max-arm-delta-deg",
        str(args.max_arm_delta_deg),
        "--hand-err-limit",
        str(args.hand_err_limit),
        "--ik-attempts",
        str(args.ik_attempts),
        "--ik-jitter",
        str(args.ik_jitter),
        "--output",
        out_path,
    ]

    if runner == "python":
        if not os.path.isfile(python_bin):
            print(f"[step2] python interpreter not found: {python_bin}", file=sys.stderr)
            return 2
        cmd = [python_bin] + base_cmd
    else:
        if shutil.which(args.uv_bin) is None:
            print(f"[step2] uv not found in PATH: {args.uv_bin}", file=sys.stderr)
            return 2
        cmd = [args.uv_bin, "run"] + base_cmd

    if args.orient_mode == "dir":
        cmd += ["--orient-dir", *(str(v) for v in args.orient_dir)]
    if args.freeze_wrist_yaw:
        cmd.append("--freeze-wrist-yaw")

    print(f"[step2] side={args.side} target=({x:.6f}, {y:.6f}, {z:.6f})")
    print(f"[step2] uv project: {args.uv_project}")
    print(f"[step2] runner: {runner}")
    print("[step2] cmd:")
    print("  " + shlex.join(cmd))

    if args.print_only:
        return 0

    started = time.time()
    proc = subprocess.run(cmd, cwd=args.uv_project)
    dt = time.time() - started
    print(f"[step2] finished with code {proc.returncode} in {dt:.2f}s")
    if proc.returncode == 0:
        print(f"[step2] output: {out_path}")
        if not args.no_archive and os.path.isfile(out_path):
            capture_tag = _latest_capture_tag(args.dataset_dir)
            archived = _archive_copy(out_path, args.archive_dir, args.side, capture_tag)
            print(f"[step2] archive: {archived}")
    return proc.returncode


if __name__ == "__main__":
    raise SystemExit(main())
