#!/usr/bin/env python3
# Управление Inspire Hand позами по кнопкам Unitree joystick.
# Не управляет arm_sdk, плечами, локтями, талией. Только кисть по TCP.

import argparse
import io
import struct
import sys
import time
import threading

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

from RH56DFTP.RH56DFTP_TCP import RH56DFTP_TCP
from Register.RegisterKey.ftp_registers_keys import (
    POS_SET_0, POS_SET_1, POS_SET_2, POS_SET_3, POS_SET_4, POS_SET_5
)

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8")
sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8")


FINGER_NAMES = {
    0: "Мизинец",
    1: "Безымянный",
    2: "Средний",
    3: "Указательный",
    4: "Большой сгиб",
    5: "Большой вращение",
}

FINGER_REGS = [POS_SET_0, POS_SET_1, POS_SET_2, POS_SET_3, POS_SET_4, POS_SET_5]

POSES = {
    "open": {
        "name": "Открытая ладонь",
        "positions": [0, 0, 0, 0, 0, 0],
    },
    "fist": {
        "name": "Кулак",
        "positions": [1800, 1800, 1800, 1800, 1800, 0],
    },
    "victory": {
        "name": "Победа / V",
        "positions": [1800, 1800, 0, 0, 1800, 0],
    },
    "rock": {
        "name": "Рок",
        "positions": [0, 1800, 1800, 0, 1800, 0],
    },
    "thumb": {
        "name": "Большой палец вверх",
        "positions": [1800, 1800, 1800, 1800, 0, 0],
    },
    "point": {
        "name": "Указание",
        "positions": [1800, 1800, 1800, 0, 1800, 0],
    },
    "ok": {
        "name": "OK",
        "positions": [0, 0, 0, 1200, 850, 2000],
    },
}


class RemoteButtons:
    def __init__(self):
        self.R1 = self.L1 = self.Start = self.Select = 0
        self.R2 = self.L2 = self.F1 = self.F3 = 0
        self.A = self.B = self.X = self.Y = 0
        self.Up = self.Down = self.Left = self.Right = 0

    def update(self, wireless_remote):
        """
        Unitree wireless_remote обычно bytes/array.
        По официальному примеру кнопки лежат в data[2] и data[3].
        """
        if wireless_remote is None:
            return False

        data = bytes(wireless_remote)
        if len(data) < 4:
            return False

        data1 = data[2]
        data2 = data[3]

        self.R1 = (data1 >> 0) & 1
        self.L1 = (data1 >> 1) & 1
        self.Start = (data1 >> 2) & 1
        self.Select = (data1 >> 3) & 1
        self.R2 = (data1 >> 4) & 1
        self.L2 = (data1 >> 5) & 1
        self.F1 = (data1 >> 6) & 1
        self.F3 = (data1 >> 7) & 1

        self.A = (data2 >> 0) & 1
        self.B = (data2 >> 1) & 1
        self.X = (data2 >> 2) & 1
        self.Y = (data2 >> 3) & 1
        self.Up = (data2 >> 4) & 1
        self.Right = (data2 >> 5) & 1
        self.Down = (data2 >> 6) & 1
        self.Left = (data2 >> 7) & 1

        return True


class HandRemoteController:
    def __init__(self, hand_ip, hand_port, movement_delay=0.06):
        self.buttons = RemoteButtons()
        self.lock = threading.Lock()
        self.remote_seen = False

        self.hand = RH56DFTP_TCP(host=hand_ip, port=hand_port)
        self.movement_delay = float(movement_delay)

        self.last_combo = None
        self.last_fire_t = 0.0
        self.cooldown_s = 0.45
        self.running = True

    def on_lowstate(self, msg: LowState_):
        with self.lock:
            ok = self.buttons.update(msg.wireless_remote)
            if ok:
                self.remote_seen = True

    def get_combo(self):
        b = self.buttons

        # Select + Start завершает скрипт
        if b.Select and b.Start:
            return "exit"

        # Простые кнопки без комбинаций
        if b.A:
            return "open"
        if b.B:
            return "fist"
        if b.X:
            return "ok"
        if b.Y:
            return "thumb"

        return None

    def move_finger(self, finger_id, position):
        self.hand.set(FINGER_REGS[finger_id], int(position))
        time.sleep(self.movement_delay)

    def execute_pose(self, pose_key):
        if pose_key not in POSES:
            return

        pose = POSES[pose_key]
        positions = pose["positions"]

        print(f"\n[HAND] pose: {pose['name']} -> {positions}")

        if pose_key == "ok":
            # Более стабильный порядок для OK:
            # открыть 0-2, затем повернуть большой, согнуть большой, подвести указательный.
            self.move_finger(0, 0)
            self.move_finger(1, 0)
            self.move_finger(2, 0)

            try:
                self.hand.set("SPEED_SET(3)", 220)
                self.hand.set("SPEED_SET(4)", 220)
                self.hand.set("SPEED_SET(5)", 180)
                self.hand.set("FORCE_SET(3)", 400)
                self.hand.set("FORCE_SET(4)", 550)
                self.hand.set("FORCE_SET(5)", 350)
            except Exception as e:
                print(f"[WARN] speed/force setup skipped: {e}")

            self.move_finger(5, positions[5])
            time.sleep(0.12)
            self.move_finger(4, positions[4])
            time.sleep(0.12)
            self.move_finger(3, positions[3])
            return

        for i, pos in enumerate(positions):
            self.move_finger(i, pos)

    def poll(self):
        now = time.monotonic()

        with self.lock:
            combo = self.get_combo()
            remote_seen = self.remote_seen

        if not remote_seen:
            return

        # Срабатывание только при новом нажатии, чтобы удержание кнопки не спамило позу.
        if combo is not None and combo != self.last_combo:
            if now - self.last_fire_t >= self.cooldown_s:
                self.last_fire_t = now

                if combo == "exit":
                    print("\n[HAND] Select + Start: exit")
                    self.running = False
                    return

                self.execute_pose(combo)

        self.last_combo = combo

    def close(self):
        try:
            self.hand.close()
        except Exception:
            pass


def main():
    p = argparse.ArgumentParser()
    p.add_argument("iface", nargs="?", default=None, help="Например eth0")
    p.add_argument("--hand-ip", default="192.168.123.211")
    p.add_argument("--hand-port", type=int, default=6000)
    p.add_argument("--remote-topic", default="rt/lf/lowstate",
                   help="Если не ловит кнопки, попробуй rt/lowstate")
    p.add_argument("--movement-delay", type=float, default=0.06)
    p.add_argument("--reset-on-start", action="store_true")
    args = p.parse_args()

    print("[INIT] DDS init...")
    if args.iface:
        ChannelFactoryInitialize(0, args.iface)
    else:
        ChannelFactoryInitialize(0)

    print(f"[INIT] hand connect {args.hand_ip}:{args.hand_port}...")
    ctrl = HandRemoteController(
        hand_ip=args.hand_ip,
        hand_port=args.hand_port,
        movement_delay=args.movement_delay,
    )

    sub = ChannelSubscriber(args.remote_topic, LowState_)
    sub.Init(ctrl.on_lowstate, 10)

    print(f"[INIT] subscribed remote topic: {args.remote_topic}")

    if args.reset_on_start:
        ctrl.execute_pose("open")

    print("""
[READY] Кнопки:
  A -> открытая ладонь
  B -> кулак
  X -> OK
  Y -> лайк / большой палец вверх

  Select + Start -> выход
""")

    try:
        while ctrl.running:
            ctrl.poll()
            time.sleep(0.03)
    except KeyboardInterrupt:
        print("\n[EXIT] KeyboardInterrupt")
    finally:
        try:
            sub.Close()
        except Exception:
            pass
        ctrl.close()
        print("[EXIT] closed")


if __name__ == "__main__":
    main()
