#!/usr/bin/env python3
import argparse
import os
import subprocess
import threading
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_


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

    p.add_argument("--up-json", default="29.json")
    p.add_argument("--left-right-json", default="35.json")

    p.add_argument("--kp", type=float, default=60.0)
    p.add_argument("--kd", type=float, default=1.5)
    p.add_argument("--kp-waist", type=float, default=250.0)
    p.add_argument("--kd-waist", type=float, default=6.0)

    # Для опасного 35 можно временно замедлить:
    p.add_argument("--duration-scale-35", type=float, default=1.0)

    p.add_argument("--sequence-timeout", type=float, default=1.5)
    args = p.parse_args()

    ChannelFactoryInitialize(0, args.iface)

    remote = RemoteReader()
    sub = ChannelSubscriber(args.remote_topic, LowState_)
    sub.Init(remote.on_lowstate, 10)

    dataset_dir = os.path.abspath(args.dataset_dir)
    player = os.path.abspath(args.player)

    current_proc = None
    prev = {}
    left_time = 0.0
    last_warn = 0.0

    def edge(btns, name):
        return bool(btns.get(name, 0)) and not bool(prev.get(name, 0))

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
            "--kp", str(args.kp),
            "--kd", str(args.kd),
            "--kp-waist", str(args.kp_waist),
            "--kd-waist", str(args.kd_waist),
            "--dataset-duration-max", "120",
            "--skip-empty-on-enter",
            "--enable-waist",
        ]

        if duration_scale != 1.0:
            cmd += ["--dataset-duration-scale", str(duration_scale)]

        print("\n[LAUNCH]", label)
        print(" ".join(cmd))

        # g1_json_upper.py спрашивает Press Enter — сразу подаем Enter.
        current_proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            text=True,
        )
        try:
            current_proc.stdin.write("\n")
            current_proc.stdin.flush()
        except Exception:
            pass

    print("[READY] joystick_launch_json.py")
    print("  Up              -> 29.json")
    print("  Left then Right -> 35.json")
    print("  Select + Start  -> exit")
    print(f"  remote topic    -> {args.remote_topic}")
    print()
    print("[IMPORTANT] Этот файл НЕ управляет arm_sdk сам. Он только запускает твой g1_json_upper.py.")

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

            if edge(btns, "Up"):
                launch("Up -> 29.json", args.up_json, duration_scale=1.0)

            if edge(btns, "Left"):
                left_time = now
                print("[SEQ] Left нажата. Теперь нажми Right.")

            if edge(btns, "Right"):
                if left_time > 0 and (now - left_time) <= args.sequence_timeout:
                    left_time = 0.0
                    launch("Left then Right -> 35.json", args.left_right_json, duration_scale=args.duration_scale_35)
                else:
                    print("[SEQ] Right без предварительной Left или таймаут.")

            prev = btns
            time.sleep(0.03)

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
