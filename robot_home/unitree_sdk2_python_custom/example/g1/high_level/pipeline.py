#!/usr/bin/env python3

"""
Схема пайплана
1) step запуск камеры и отправка данных на сервер для разметки бокса с тем что ищем. 
возвращает сразу xyz координаты обсчитанные из глубины и ргб 

pkill -f realsense2_camera

ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  align_depth.enable:=true \
  depth_module.profile:="640,480,15" \
  rgb_camera.profile:="640,480,15"
  
ОБЯЗАТЕЛЬНО ПЕРЕД ЭТИМ ЗАЙТИ НА СЕРВЕР

И сделать запуск вотчера (приемника данных с робота)

(base) root@NeuroMorphMIPT:/home/arutiunyan_ag/owlv2_test# source owlv2_env/bin/activate
(owlv2_env) (base) root@NeuroMorphMIPT:/home/arutiunyan_ag/owlv2_test# 

python3 owlv2_watch_server.py \
  --watch-dir /home/arutiunyan_ag/owlv2_test/dataset_targets \
  --result-dir /home/arutiunyan_ag/owlv2_test/dataset_results \
  --query-path /home/arutiunyan_ag/owlv2_test/button_query.png \
  --model-path /home/arutiunyan_ag/owlv2_test/models/owlv2-large-patch14 \
  --hf-cache /home/arutiunyan_ag/owlv2_test/models/hf_cache \
  --poll-sec 0.2 \
  --min-age-sec 0.2
П.С чтобы каждый раз пароль не вводить для ssh 
ssh-keygen -t ed25519 -C "unitree-pipeline"
ssh-copy-id arutiunyan_ag@192.168.1.89
ssh arutiunyan_ag@192.168.1.89


Дальше уже можно отправлять на него

python3 1step_detect_xyz.py \
  --detector server \
  --server-host 192.168.1.89 \
  --server-user arutiunyan_ag \
  --server-in-dir /home/arutiunyan_ag/owlv2_test/dataset_targets \
  --server-out-dir /home/arutiunyan_ag/owlv2_test/dataset_results \
  --target-dx-m -0.08 \
  --target-dy-m -0.025 \
  --target-dz-m 0.1


2) step запуск расчета ИК для создания json с суставами для запуска движения робота

python3 2step_xyz_to_angles_IK.py --side right --runner auto --hand-err-limit 0.05

python3 2step_xyz_to_angles_IK.py --side right --runner auto --hand-err-limit 0.06


3) step добавляет толчок рукой (нажатие) в json и запускает json на роботе

python3 3_0_step_add_push_frame.py

python3 3_1_sdk_hands.py --dataset-path ./pipeline/ik_records/current_target_right.json \
--enable-wrist-py --kp-waist 250 --kd-waist 6


"""

import os
import shlex
import signal
import subprocess
import sys
import time
import json
from glob import glob


def _fmt(cmd):
    return " ".join(shlex.quote(x) for x in cmd)


def run_step(name, cmd, cwd):
    print(f"\n==> {name}")
    print(f"$ {_fmt(cmd)}")
    result = subprocess.run(cmd, cwd=cwd)
    if result.returncode != 0:
        print(f"\nERROR: step '{name}' failed (exit code {result.returncode})", file=sys.stderr)
        return False
    return True


def run_cleanup_realsense(cwd):
    cmd = ["pkill", "-f", "realsense2_camera"]
    print("\n==> cleanup old realsense")
    print(f"$ {_fmt(cmd)}")
    result = subprocess.run(cmd, cwd=cwd)
    if result.returncode in (0, 1):
        return True
    print(f"\nERROR: cleanup failed (exit code {result.returncode})", file=sys.stderr)
    return False


def stop_process(proc, name):
    if proc is None or proc.poll() is not None:
        return
    print(f"\nStopping background process: {name}")
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


def detector_supports_auto_flags(cwd):
    cmd = ["python3", "1step_detect_xyz.py", "--help"]
    result = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    if result.returncode != 0:
        return False
    help_text = (result.stdout or "") + "\n" + (result.stderr or "")
    return "--auto-save-and-exit" in help_text and "--auto-wait-sec" in help_text


def resolve_hands_runner(cwd):
    candidates = [
        "3_1_step_sdk_hands.py",
        "3step_sdk_hands.py",
        "3_1_sdk_hands.py",
    ]
    for name in candidates:
        if os.path.isfile(os.path.join(cwd, name)):
            return name
    return None


def print_detected_button_xyz(cwd):
    dataset_dir = os.path.join(cwd, "dataset_rgbd")
    meta_files = glob(os.path.join(dataset_dir, "*_meta.json"))
    if not meta_files:
        print("WARNING: no *_meta.json found in dataset_rgbd; cannot print detected XYZ.")
        return

    latest_meta = max(meta_files, key=os.path.getmtime)
    try:
        with open(latest_meta, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception as exc:
        print(f"WARNING: failed to read meta file {latest_meta}: {exc}")
        return

    xyz_cam = data.get("xyz_cam_m")
    xyz_pelvis = data.get("xyz_pelvis_m")
    bbox = data.get("bbox_xywh")

    def _fmt_xyz_value(v):
        try:
            return f"{float(v):.4f}"
        except Exception:
            return str(v)

    print("\n==> detection summary")
    print(f"meta: {latest_meta}")
    if isinstance(xyz_cam, dict):
        print(
            "button xyz (camera, m): "
            f"x={_fmt_xyz_value(xyz_cam.get('x'))}, "
            f"y={_fmt_xyz_value(xyz_cam.get('y'))}, "
            f"z={_fmt_xyz_value(xyz_cam.get('z'))}"
        )
    else:
        print("button xyz (camera, m): not found")

    if isinstance(xyz_pelvis, dict):
        print(
            "button xyz (pelvis, m): "
            f"x={_fmt_xyz_value(xyz_pelvis.get('x'))}, "
            f"y={_fmt_xyz_value(xyz_pelvis.get('y'))}, "
            f"z={_fmt_xyz_value(xyz_pelvis.get('z'))}"
        )
    else:
        print("button xyz (pelvis, m): not found")

    if isinstance(bbox, dict):
        print(f"bbox xywh: {bbox}")


def main():
    cwd = os.path.dirname(os.path.abspath(__file__))
    hands_runner = resolve_hands_runner(cwd)
    if hands_runner is None:
        print(
            "ERROR: hands runner script not found. "
            "Expected one of: 3_1_step_sdk_hands.py, 3step_sdk_hands.py, 3_1_sdk_hands.py",
            file=sys.stderr,
        )
        return 1

    has_auto_flags = detector_supports_auto_flags(cwd)
    if not has_auto_flags:
        print(
            "ERROR: 1step_detect_xyz.py does not support --auto-save-and-exit/--auto-wait-sec.\n"
            "Update this file on the running machine to the new version, "
            "or remove auto flags and use manual 's' then 'q'.",
            file=sys.stderr,
        )
        return 1

    first_step_bg = None
    try:
        if not run_cleanup_realsense(cwd):
            return 1

        print("\n==> start realsense in background")
        ros_cmd = [
            "ros2",
            "launch",
            "realsense2_camera",
            "rs_launch.py",
            "enable_color:=true",
            "align_depth.enable:=true",
            "depth_module.profile:=640,480,15",
            "rgb_camera.profile:=640,480,15",
        ]
        print(f"$ {_fmt(ros_cmd)}")
        first_step_bg = subprocess.Popen(ros_cmd, cwd=cwd, start_new_session=True)
        time.sleep(2.0)
        if first_step_bg.poll() is not None:
            print("ERROR: realsense launch exited immediately", file=sys.stderr)
            return 1

        if not run_step(
            "1step_detect_xyz",
            [
                "python3",
                "1step_detect_xyz.py",
                "--detector",
                "server",
                "--server-host",
                "192.168.1.89",
                "--server-user",
                "arutiunyan_ag",
                "--server-key-path",
                os.path.expanduser("~/.ssh/id_ed25519"),
                "--server-in-dir",
                "/home/arutiunyan_ag/owlv2_test/dataset_targets",
                "--server-out-dir",
                "/home/arutiunyan_ag/owlv2_test/dataset_results",
                "--no-server-fallback-manual",
                "--target-dx-m",
                "-0.08",
                "--target-dy-m",
                "-0.025",
                "--target-dz-m",
                "0.1",
                "--auto-save-and-exit",
                "--auto-wait-sec",
                "1.0",
            ],
            cwd,
        ):
            return 1
        print_detected_button_xyz(cwd)
    finally:
        if first_step_bg is not None and first_step_bg.poll() is None:
            os.killpg(os.getpgid(first_step_bg.pid), signal.SIGTERM)
            stop_process(first_step_bg, "ros2 realsense")

    if not run_step(
        "2step IK strict",
        ["python3", "2step_xyz_to_angles_IK.py", "--side", "right", "--runner", "auto", "--hand-err-limit", "0.05"],
        cwd,
    ):
        return 1

    if not run_step(
        "2step IK relaxed",
        ["python3", "2step_xyz_to_angles_IK.py", "--side", "right", "--runner", "auto", "--hand-err-limit", "0.06"],
        cwd,
    ):
        return 1

    if not run_step("3_0 add push frame", ["python3", "3_0_step_add_push_frame.py"], cwd):
        return 1

    if not run_step(
        "3_1 run hands",
        [
            "python3",
            hands_runner,
            "--dataset-path",
            "./pipeline/ik_records/current_target_right.json",
            "--enable-wrist-py",
            "--kp-waist",
            "250",
            "--kd-waist",
            "6",
        ],
        cwd,
    ):
        return 1

    print("\nPipeline completed successfully.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

