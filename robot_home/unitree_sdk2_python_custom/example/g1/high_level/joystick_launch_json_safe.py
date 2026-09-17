#!/usr/bin/env python3
import argparse
import os
import subprocess
import threading
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_


"""

python3 joystick_launch_json_safe.py eth0 \
  --dataset-dir dataset_100 \
  --json-a 29.json \
  --json-b 35.json \
  --json-down 47.json \
  --duration-scale-b 1.0 \
  --kp 60 \
  --kd 1.5 \
  --kp-waist 250 \
  --kd-waist 6

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

    p.add_argument("--json-a", default="29.json")
    p.add_argument("--json-b", default="35.json")
    p.add_argument("--json-down", default="47.json")

    p.add_argument("--kp", type=float, default=60.0)
    p.add_argument("--kd", type=float, default=1.5)
    p.add_argument("--kp-waist", type=float, default=250.0)
    p.add_argument("--kd-waist", type=float, default=6.0)

    # 1.0 = как в JSON, без замедления
    p.add_argument("--duration-scale-b", type=float, default=1.0)

    # ждать отпускания кнопки перед запуском
    p.add_argument("--launch-after-release-sec", type=float, default=0.35)

    args = p.parse_args()

    ChannelFactoryInitialize(0, args.iface)

    remote = RemoteReader()
    sub = ChannelSubscriber(args.remote_topic, LowState_)
    sub.Init(remote.on_lowstate, 10)

    dataset_dir = os.path.abspath(args.dataset_dir)
    player = os.path.abspath(args.player)

    current_proc = None
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

    def is_busy():
        nonlocal current_proc
        if current_proc is None:
            return False
        if current_proc.poll() is None:
            return True
        current_proc = None
        return False

    def launch(label, json_name, duration_scale=1.0):
        nonlocal current_proc

        if is_busy():
            print(f"[BUSY] Сейчас уже играет JSON. Игнорирую: {label}")
            return

        path = os.path.join(dataset_dir, json_name)
        if not os.path.isfile(path):
            print(f"[ERROR] Нет файла: {path}")
            return

        cmd = [
            "python3", player, args.iface,
            "--dataset-path", path,
            "--enable-wrist-py",
            "--enable-waist",
            "--kp", str(args.kp),
            "--kd", str(args.kd),
            "--kp-waist", str(args.kp_waist),
            "--kd-waist", str(args.kd_waist),
            "--dataset-duration-max", "120",
        ]

        if duration_scale != 1.0:
            cmd += ["--dataset-duration-scale", str(duration_scale)]

        print("\n[LAUNCH]", label)
        print(" ".join(cmd))

        current_proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            text=True,
        )

        # g1_json_upper.py спрашивает Press Enter — подаем Enter и закрываем stdin.
        try:
            current_proc.stdin.write("\n")
            current_proc.stdin.flush()
            current_proc.stdin.close()
        except Exception:
            pass

    print("[READY] joystick_launch_json_safe.py")
    print("  A              -> 29.json")
    print("  B              -> 35.json")
    print("  Down           -> 47.json")
    print("  Select + Start -> exit")
    print(f"  remote topic   -> {args.remote_topic}")
    print()
    print("[IMPORTANT] Используется только Down из D-pad: Down -> 47.json. Left/Right/Up не используются.")

    try:
        while True:
            btns, seen = remote.snapshot()
            now = time.monotonic()

            if not seen and now - last_warn > 2.0:
                last_warn = now
                print(f"[WARN] Нет данных джойстика на {args.remote_topic}")

            if btns.get("Select") and btns.get("Start"):
                print("[EXIT] Select + Start")
                break

            if pending is None:
                if edge(btns, "A"):
                    pending = ("A -> 29.json", args.json_a, 1.0)
                    release_since = 0.0
                    print("[PENDING] A detected. Release buttons to launch 29.json.")

                elif edge(btns, "B"):
                    pending = ("B -> 35.json", args.json_b, args.duration_scale_b)
                    release_since = 0.0
                    print("[PENDING] B detected. Release buttons to launch 35.json.")

                elif edge(btns, "Down"):
                    pending = ("Down -> 47.json", args.json_down, 1.0)
                    release_since = 0.0
                    print("[PENDING] Down detected. Release buttons to launch 47.json.")

            if pending is not None:
                if any_pressed(btns):
                    release_since = 0.0
                else:
                    if release_since == 0.0:
                        release_since = now
                    elif now - release_since >= args.launch_after_release_sec:
                        label, json_name, scale = pending
                        pending = None
                        release_since = 0.0
                        launch(label, json_name, duration_scale=scale)

            prev = btns
            time.sleep(0.03)

    except KeyboardInterrupt:
        print("\n[EXIT] KeyboardInterrupt")

    finally:
        try:
            sub.Close()
        except Exception:
            pass

        if current_proc is not None and current_proc.poll() is None:
            print("[EXIT] Оставляю текущий g1_json_upper.py завершиться сам.")
        print("[DONE]")


if __name__ == "__main__":
    main()
