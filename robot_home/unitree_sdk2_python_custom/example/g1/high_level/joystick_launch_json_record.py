#!/usr/bin/env python3
import argparse
import json
import os
import shlex
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

from RH56DFTP.RH56DFTP_TCP import RH56DFTP_TCP

OPEN_POSE = [0, 0, 0, 0, 0, 0]
FIST_POSE = [1800, 1800, 1800, 1800, 2000, 2000]

"""
python3 joystick_launch_json_record.py eth0 \
  --dataset-dir dataset_100 \
  --json-a 29.json \
  --json-b 35.json \
  --json-down 47.json \
  --duration-scale-a 1.0 \
  --duration-scale-b 1.0 \
  --duration-scale-down 1.0 \
  --kp 60 \
  --kd 1.5 \
  --kp-waist 250 \
  --kd-waist 6 \
  --record-out-dir ./actual_motion_logs \
  --record-rate-hz 50 \
  --record-target-frame pelvis
  
  
python3 joystick_launch_json_record.py eth0 \
  --dataset-dir dataset_100 \
  --json-a 888.json \
  --json-b 35.json \
  --json-down 47.json \
  --duration-scale-a 1.0 \
  --duration-scale-b 1.0 \
  --duration-scale-down 1.0 \
  --kp 60 \
  --kd 1.5 \
  --kp-waist 250 \
  --kd-waist 6 \
  --record-out-dir ./actual_motion_logs \
  --record-rate-hz 50 \
  --record-target-frame pelvis  
  
  
python3 joystick_launch_json_record.py eth0 \
  --dataset-dir dataset_100 \
  --json-a 00.json \
  --json-b 1111.json \
  --json-down 0.json \
  --duration-scale-a 1.0 \
  --duration-scale-b 1.0 \
  --duration-scale-down 1.0 \
  --kp 60 \
  --kd 1.5 \
  --kp-waist 250 \
  --kd-waist 6 \
  --record-out-dir ./actual_motion_logs \
  --record-rate-hz 50 \
  --record-target-frame pelvis   
  
python3 joystick_launch_json_record.py eth0 \
  --dataset-dir dataset_100 \
  --json-a give_apple.json \
  --json-b apple.json \
  --json-down 0.json \
  --duration-scale-a 1.0 \
  --duration-scale-b 1.0 \
  --duration-scale-down 1.0 \
  --kp 60 \s
  --kd 1.5 \
  --kp-waist 250 \
  --kd-waist 6 \
  --record-out-dir ./actual_motion_logs \
  --record-rate-hz 50 \
  --record-target-frame pelvis  
  
python3 joystick_launch_json_record.py eth0 \
  --dataset-dir dataset_100 \
  --json-a wave_right000.json \
  --json-b hold_box.json \
  --json-down 0.json \
  --duration-scale-a 1.0 \
  --duration-scale-b 1.0 \
  --duration-scale-down 1.0 \
  --kp 60 \
  --kd 1.5 \
  --kp-waist 250 \
  --kd-waist 6 \
  --record-out-dir ./actual_motion_logs \
  --record-rate-hz 50 \
  --record-target-frame pelvis      
  
python3 joystick_launch_json_record.py eth0 \
  --dataset-dir dataset_100 \
  --json-a hold_apple_then_roll_it.json \
  --json-b hold_apple.json \
  --json-down 0noting.json \
  --duration-scale-a 1.0 \
  --duration-scale-b 1.0 \
  --duration-scale-down 1.0 \
  --kp 60 \
  --kd 1.5 \
  --kp-waist 250 \
  --kd-waist 6 \
  --record-out-dir ./actual_motion_logs \
  --record-rate-hz 50 \
  --record-target-frame pelvis   
"""

class RemoteButtons:
    def __init__(self):
        self.Start = self.Select = 0
        self.Up = self.Down = self.Left = self.Right = 0
        self.A = self.B = self.X = self.Y = 0

    def update(self, wireless_remote):
        try:
            data = bytes(wireless_remote)
        except Exception:
            return False

        if len(data) < 4:
            return False

        data1 = data[2]
        data2 = data[3]

        self.Start = (data1 >> 2) & 1
        self.Select = (data1 >> 3) & 1

        self.A = (data2 >> 0) & 1
        self.B = (data2 >> 1) & 1
        self.X = (data2 >> 2) & 1
        self.Y = (data2 >> 3) & 1

        self.Up = (data2 >> 4) & 1
        self.Right = (data2 >> 5) & 1
        self.Down = (data2 >> 6) & 1
        self.Left = (data2 >> 7) & 1

        return True

    def snapshot(self):
        return {
            "Start": self.Start,
            "Select": self.Select,
            "Up": self.Up,
            "Down": self.Down,
            "Left": self.Left,
            "Right": self.Right,
            "A": self.A,
            "B": self.B,
            "X": self.X,
            "Y": self.Y,
        }


class RemoteReader:
    def __init__(self):
        self.buttons = RemoteButtons()
        self.lock = threading.Lock()
        self.seen = False

    def on_lowstate(self, msg):
        with self.lock:
            if self.buttons.update(msg.wireless_remote):
                self.seen = True

    def snapshot(self):
        with self.lock:
            return self.buttons.snapshot(), self.seen


def main():
    p = argparse.ArgumentParser()
    p.add_argument("iface", nargs="?", default="eth0")
    p.add_argument("--remote-topic", default="rt/lf/lowstate")
    p.add_argument("--dataset-dir", default="dataset_100")
    p.add_argument("--player", default="./g1_json_upper.py")
    p.add_argument("--recorder", default="./record_actual_motion_ros.py")

    p.add_argument("--json-a", default="29.json")
    p.add_argument("--json-b", default="35.json")
    p.add_argument("--json-down", default="47.json")

    p.add_argument("--kp", type=float, default=60.0)
    p.add_argument("--kd", type=float, default=1.5)
    p.add_argument("--kp-waist", type=float, default=250.0)
    p.add_argument("--kd-waist", type=float, default=6.0)

    p.add_argument("--duration-scale-a", type=float, default=1.0)
    p.add_argument("--duration-scale-b", type=float, default=1.0)
    p.add_argument("--duration-scale-down", type=float, default=1.0)

    p.add_argument("--record-out-dir", default="./actual_motion_logs")
    p.add_argument("--record-rate-hz", type=float, default=50.0)
    p.add_argument("--record-warmup-sec", type=float, default=0.25)
    p.add_argument("--record-target-frame", default="pelvis")
    p.add_argument("--record-links", default="")
    p.add_argument("--no-record", action="store_true")

    p.add_argument("--hand-ip-right", default="192.168.123.211")
    p.add_argument("--hand-port", type=int, default=6000)
    p.add_argument("--hand-speed", type=int, default=180)
    p.add_argument("--hand-force", type=int, default=300)

    p.add_argument("--launch-after-release-sec", type=float, default=0.35)
    args = p.parse_args()

    hand = None
    try:
        hand = RH56DFTP_TCP(
            host=args.hand_ip_right,
            port=args.hand_port,
        )
        for i in range(6):
            hand.set(f"SPEED_SET({i})", int(args.hand_speed))
            hand.set(f"FORCE_SET({i})", int(args.hand_force))
        print(f"[HAND] Connected right hand: {args.hand_ip_right}:{args.hand_port}")
    except Exception as exc:
        hand = None
        print(f"[HAND ERROR] Right hand connection failed: {exc}")

    def set_right_hand_pose(pose):
        if hand is None:
            print("[HAND ERROR] Right hand is not connected")
            return
        try:
            for i, value in enumerate(pose):
                hand.set(f"POS_SET({i})", int(value))
            print(f"[HAND] Pose sent: {pose}")
        except Exception as exc:
            print(f"[HAND ERROR] {exc}")

    ChannelFactoryInitialize(0, args.iface)

    remote = RemoteReader()
    sub = ChannelSubscriber(args.remote_topic, LowState_)
    sub.Init(remote.on_lowstate, 10)

    dataset_dir = Path(args.dataset_dir).resolve()
    player = Path(args.player).resolve()
    recorder = Path(args.recorder).resolve()

    current_proc = None
    current_rec_proc = None
    prev = {}
    pending = None
    release_since = 0.0
    last_warn = 0.0

    def edge(btns, name):
        return bool(btns.get(name, 0)) and not bool(prev.get(name, 0))

    def any_pressed(btns):
        return any(bool(btns.get(k, 0)) for k in [
            "Start", "Select", "Up", "Down", "Left", "Right", "A", "B", "X", "Y"
        ])

    def stop_recorder():
        nonlocal current_rec_proc
        if current_rec_proc is not None and current_rec_proc.poll() is None:
            print("[REC] stopping recorder...")
            current_rec_proc.terminate()
            try:
                current_rec_proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                current_rec_proc.kill()
        current_rec_proc = None

    def update_finished_process():
        nonlocal current_proc
        if current_proc is not None and current_proc.poll() is not None:
            print("[PLAYER] finished")
            current_proc = None
            stop_recorder()

    def is_busy():
        update_finished_process()
        return current_proc is not None and current_proc.poll() is None

    def skip_current_empty():
        if current_proc is None or current_proc.poll() is not None:
            print("[SKIP] JSON сейчас не выполняется")
            return

        if current_proc.stdin is None:
            print("[SKIP ERROR] stdin плеера недоступен")
            return

        try:
            current_proc.stdin.write("\n")
            current_proc.stdin.flush()
            print("[SKIP] Up -> запрос пропуска текущего пустого frame")
        except (BrokenPipeError, OSError) as exc:
            print(f"[SKIP ERROR] {exc}")

    def start_recorder(run_name):
        nonlocal current_rec_proc

        if args.no_record:
            return True

        if not recorder.is_file():
            print(f"[REC ERROR] recorder not found: {recorder}")
            return False

        out_root = Path(args.record_out_dir).resolve()
        out_root.mkdir(parents=True, exist_ok=True)

        log_path = out_root / f"{run_name}_recorder_stdout.log"

        py_args = [
            "/usr/bin/python3",
            str(recorder),
            "--out-dir", str(out_root),
            "--run-name", run_name,
            "--target-frame", args.record_target_frame,
            "--rate-hz", str(args.record_rate_hz),
        ]

        if args.record_links.strip():
            py_args += ["--links", args.record_links.strip()]

        # Важно: команды разделены через ; и python запускается через exec.
        shell_cmd = (
            "source /opt/ros/humble/setup.bash; "
            "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; "
            "export PYTHONNOUSERSITE=1; "
            "exec " + " ".join(shlex.quote(x) for x in py_args)
        )

        print(f"[REC] start: {run_name}")
        print(f"[REC] log: {log_path}")

        log_f = open(log_path, "w", encoding="utf-8")
        current_rec_proc = subprocess.Popen(
            ["bash", "-lc", shell_cmd],
            stdout=log_f,
            stderr=subprocess.STDOUT,
            text=True,
        )

        # Даём recorder чуть-чуть времени подписаться на /joint_states и /tf.
        time.sleep(max(0.0, float(args.record_warmup_sec)))

        if current_rec_proc.poll() is not None:
            print(f"[REC ERROR] recorder exited immediately. Check log: {log_path}")
            current_rec_proc = None
            return False

        return True

    def launch(label, json_name, duration_scale=1.0):
        nonlocal current_proc

        if is_busy():
            print(f"[BUSY] Сейчас уже играет JSON. Игнорирую: {label}")
            return

        path = dataset_dir / json_name
        if not path.is_file():
            print(f"[ERROR] Нет файла: {path}")
            return

        stem = path.stem
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_name = f"from_{stem}_{ts}"

        if not start_recorder(run_name):
            print("[ABORT] Recorder failed to start, motion will NOT be launched.")
            return

        cmd = [
            "python3", str(player), args.iface,
            "--dataset-path", str(path),
            "--enable-wrist-py",
            "--enable-waist",
            "--kp", str(args.kp),
            "--kd", str(args.kd),
            "--kp-waist", str(args.kp_waist),
            "--kd-waist", str(args.kd_waist),
            "--dataset-duration-max", "120",
            "--skip-empty-on-enter",
        ]

        if duration_scale != 1.0:
            cmd += ["--dataset-duration-scale", str(duration_scale)]

        # Сохраняем параметры запуска рядом с фактическими CSV.
        # Это нужно, чтобы потом сравнивать фактическое движение с ожидаемым JSON
        # и точно знать, с какими kp/kd оно было выполнено.
        run_dir = Path(args.record_out_dir).resolve() / run_name
        run_dir.mkdir(parents=True, exist_ok=True)

        launch_meta = {
            "run_name": run_name,
            "label": label,
            "button_label": label.split("->")[0].strip() if "->" in label else label,
            "json_name": json_name,
            "json_path": str(path),
            "json_stem": stem,
            "timestamp": ts,

            "iface": args.iface,
            "player": str(player),
            "recorder": str(recorder),

            "arm_control": {
                "kp": float(args.kp),
                "kd": float(args.kd),
                "enable_wrist_py": True,
                "enable_waist": True,
                "dataset_duration_scale": float(duration_scale),
                "dataset_duration_max": 120.0,
            },

            "waist_control": {
                "kp_waist": float(args.kp_waist),
                "kd_waist": float(args.kd_waist),
            },

            "recording": {
                "enabled": not bool(args.no_record),
                "record_out_dir": str(Path(args.record_out_dir).resolve()),
                "record_rate_hz": float(args.record_rate_hz),
                "record_target_frame": args.record_target_frame,
                "record_links": args.record_links,
            },

            "command": cmd,
            "command_string": " ".join(cmd),
        }

        launch_meta_path = run_dir / "launch_params.json"
        launch_meta_path.write_text(
            json.dumps(launch_meta, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

        print("\n[LAUNCH]", label)
        print("run_name:", run_name)
        print("launch_params:", launch_meta_path)
        print(" ".join(cmd))

        current_proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            text=True,
        )

        try:
            # Первый Enter подтверждает запуск g1_json_upper.py.
            # stdin оставляем открытым для команды Skip по стрелке Up.
            current_proc.stdin.write("\n")
            current_proc.stdin.flush()
        except Exception:
            pass

    print("[READY] joystick_launch_json_record.py")
    print("  A              ->", args.json_a)
    print("  B              ->", args.json_b)
    print("  Down           ->", args.json_down)
    print("  Up             -> skip current empty frame")
    print("  X              -> right hand OPEN")
    print("  Y              -> right hand FIST")
    print("  Select + Start -> exit")
    print(f"  record out     -> {Path(args.record_out_dir).resolve()}")
    print(f"  remote topic   -> {args.remote_topic}")
    print()
    print("[INFO] Запись начнется ровно при запуске JSON после отпускания кнопки.")

    try:
        while True:
            update_finished_process()

            btns, seen = remote.snapshot()
            now = time.monotonic()

            if not seen and now - last_warn > 2.0:
                last_warn = now
                print(f"[WARN] Нет данных джойстика на {args.remote_topic}")

            if btns.get("Select") and btns.get("Start"):
                print("[EXIT] Select + Start")
                break

            if pending is None:
                if edge(btns, "Up"):
                    print("[SKIP] Up pressed")
                    skip_current_empty()

                elif edge(btns, "X"):
                    print("[HAND] X -> OPEN")
                    set_right_hand_pose(OPEN_POSE)

                elif edge(btns, "Y"):
                    print("[HAND] Y -> FIST")
                    set_right_hand_pose(FIST_POSE)

                elif edge(btns, "A"):
                    pending = ("A", args.json_a, args.duration_scale_a)
                    release_since = 0.0
                    print(f"[PENDING] A detected. Release buttons to launch {args.json_a}")

                elif edge(btns, "B"):
                    pending = ("B", args.json_b, args.duration_scale_b)
                    release_since = 0.0
                    print(f"[PENDING] B detected. Release buttons to launch {args.json_b}")

                elif edge(btns, "Down"):
                    pending = ("Down", args.json_down, args.duration_scale_down)
                    release_since = 0.0
                    print(f"[PENDING] Down detected. Release buttons to launch {args.json_down}")

            if pending is not None:
                if any_pressed(btns):
                    release_since = 0.0
                else:
                    if release_since == 0.0:
                        release_since = now
                    elif now - release_since >= args.launch_after_release_sec:
                        button, json_name, scale = pending
                        pending = None
                        release_since = 0.0
                        launch(f"{button} -> {json_name}", json_name, duration_scale=scale)

            prev = btns
            time.sleep(0.03)

    except KeyboardInterrupt:
        print("\n[EXIT] KeyboardInterrupt")

    finally:
        try:
            sub.Close()
        except Exception:
            pass

        if hand is not None:
            try:
                hand.close()
            except Exception:
                pass

        stop_recorder()
        print("[DONE]")


if __name__ == "__main__":
    main()
