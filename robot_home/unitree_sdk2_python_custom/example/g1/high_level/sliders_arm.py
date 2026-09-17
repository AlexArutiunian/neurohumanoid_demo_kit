#!/usr/bin/env python3
"""GUI sliders for RH56 hand position and force tuning (channels 0..5).

python sliders_arm.py --ip 192.168.123.210 --port 6000

"""

import argparse
import time
import tkinter as tk
from tkinter import ttk

from RH56DFTP.RH56DFTP_TCP import RH56DFTP_TCP
from Register.RegisterKey.ftp_registers_keys import ALL_REGISTER_NAMES

FINGER_NAMES = [
    "0: Little",
    "1: Ring",
    "2: Middle",
    "3: Index",
    "4: Thumb bend",
    "5: Thumb rotate",
]

OPEN_POSE = [0, 0, 0, 0, 0, 0]
FIST_POSE = [1800, 1800, 1800, 1800, 1800, 0]


def pick_prefix(*candidates):
    for c in candidates:
        if f"{c}(0)" in ALL_REGISTER_NAMES:
            return c
    return None


class HandSlidersApp:
    def __init__(self, root, client, args):
        self.root = root
        self.client = client
        self.args = args
        self.pos_scales = []
        self.pos_set_labels = []
        self.force_scales = []
        self.force_set_labels = []
        self.act_pos_labels = []
        self.act_force_labels = []
        self.after_id = None

        self.pos_read_prefix = pick_prefix("POS_ACT", "ANGLE_ACT", "POS_SET", "ANGLE_SET") or "POS_SET"
        self.force_read_prefix = pick_prefix("FORCE_ACT", "FORCE_SET") or "FORCE_SET"

        self.root.title(f"RH56 sliders  {args.ip}:{args.port}")
        self.root.geometry("1120x560")

        self.auto_send = tk.BooleanVar(value=True)
        self.status_var = tk.StringVar(value="Connected")
        self.group4_var = tk.IntVar(value=0)

        self._build_ui()
        self._apply_speed_force_defaults()
        self._set_pose(OPEN_POSE)
        self._schedule_readback(initial=True)

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill=tk.X)

        ttk.Label(top, text=f"IP: {self.args.ip}:{self.args.port}").pack(side=tk.LEFT)
        ttk.Checkbutton(top, text="Auto send", variable=self.auto_send).pack(side=tk.LEFT, padx=12)

        ttk.Button(top, text="Open", command=lambda: self._set_pose(OPEN_POSE)).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Fist", command=lambda: self._set_pose(FIST_POSE)).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Read now", command=self._readback_once).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Apply pos", command=self._apply_all_positions).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Apply force", command=self._apply_all_forces).pack(side=tk.LEFT, padx=4)

        ttk.Label(top, textvariable=self.status_var, foreground="#0a7d00").pack(side=tk.RIGHT)

        group = ttk.Frame(self.root, padding=(10, 0, 10, 6))
        group.pack(fill=tk.X)
        ttk.Label(group, text="Group 0..3 (little/ring/middle/index):").pack(side=tk.LEFT)
        group_scale = tk.Scale(
            group,
            from_=0,
            to=2000,
            orient=tk.HORIZONTAL,
            length=350,
            showvalue=True,
            variable=self.group4_var,
            command=self._on_group4_change,
        )
        group_scale.pack(side=tk.LEFT, padx=8)
        ttk.Button(group, text="Open 0..3", command=lambda: self._set_group4(0)).pack(side=tk.LEFT, padx=4)
        ttk.Button(group, text="Close 0..3", command=lambda: self._set_group4(1800)).pack(side=tk.LEFT, padx=4)

        body = ttk.Frame(self.root, padding=(10, 6, 10, 10))
        body.pack(fill=tk.BOTH, expand=True)

        hdr = ttk.Frame(body)
        hdr.pack(fill=tk.X)
        ttk.Label(hdr, text="Finger", width=16).grid(row=0, column=0, sticky="w")
        ttk.Label(hdr, text="Pos set", width=8).grid(row=0, column=1)
        ttk.Label(hdr, text="Pos slider", width=34).grid(row=0, column=2)
        ttk.Label(hdr, text="Force set", width=8).grid(row=0, column=3)
        ttk.Label(hdr, text="Force slider", width=34).grid(row=0, column=4)
        ttk.Label(hdr, text="Act pos", width=10).grid(row=0, column=5)
        ttk.Label(hdr, text="Act force", width=10).grid(row=0, column=6)

        for i, name in enumerate(FINGER_NAMES):
            row = ttk.Frame(body)
            row.pack(fill=tk.X, pady=3)

            ttk.Label(row, text=name, width=16).grid(row=0, column=0, sticky="w")

            pos_set_lbl = ttk.Label(row, text="0", width=8)
            pos_set_lbl.grid(row=0, column=1)
            self.pos_set_labels.append(pos_set_lbl)

            pos_scale = tk.Scale(
                row,
                from_=0,
                to=2000,
                orient=tk.HORIZONTAL,
                length=300,
                showvalue=False,
                resolution=1,
                command=lambda v, idx=i: self._on_pos_slider_change(idx, v),
            )
            pos_scale.grid(row=0, column=2, padx=6, sticky="we")
            self.pos_scales.append(pos_scale)

            force_set_lbl = ttk.Label(row, text=str(self.args.force), width=8)
            force_set_lbl.grid(row=0, column=3)
            self.force_set_labels.append(force_set_lbl)

            force_scale = tk.Scale(
                row,
                from_=0,
                to=1000,
                orient=tk.HORIZONTAL,
                length=300,
                showvalue=False,
                resolution=1,
                command=lambda v, idx=i: self._on_force_slider_change(idx, v),
            )
            force_scale.set(int(self.args.force))
            force_scale.grid(row=0, column=4, padx=6, sticky="we")
            self.force_scales.append(force_scale)

            pos_lbl = ttk.Label(row, text="-", width=10)
            pos_lbl.grid(row=0, column=5)
            self.act_pos_labels.append(pos_lbl)

            force_lbl = ttk.Label(row, text="-", width=10)
            force_lbl.grid(row=0, column=6)
            self.act_force_labels.append(force_lbl)

    def _safe_get(self, reg, default="-"):
        try:
            return self.client.get(reg)
        except Exception:
            return default

    def _safe_set(self, reg, value):
        try:
            self.client.set(reg, int(value))
            return True
        except Exception as exc:
            self.status_var.set(f"Write error: {exc}")
            return False

    def _apply_speed_force_defaults(self):
        for i in range(6):
            self._safe_set(f"SPEED_SET({i})", self.args.speed)
            self._safe_set(f"FORCE_SET({i})", self.args.force)

    def _on_pos_slider_change(self, idx, value):
        v = int(float(value))
        self.pos_set_labels[idx].configure(text=str(v))
        if self.auto_send.get():
            self._safe_set(f"POS_SET({idx})", v)

    def _on_force_slider_change(self, idx, value):
        v = int(float(value))
        self.force_set_labels[idx].configure(text=str(v))
        if self.auto_send.get():
            self._safe_set(f"FORCE_SET({idx})", v)

    def _current_pos_targets(self):
        return [int(s.get()) for s in self.pos_scales]

    def _current_force_targets(self):
        return [int(s.get()) for s in self.force_scales]

    def _apply_all_positions(self):
        vals = self._current_pos_targets()
        for i, v in enumerate(vals):
            self._safe_set(f"POS_SET({i})", v)
        self.status_var.set(f"Applied pos: {vals}")

    def _apply_all_forces(self):
        vals = self._current_force_targets()
        for i, v in enumerate(vals):
            self._safe_set(f"FORCE_SET({i})", v)
        self.status_var.set(f"Applied force: {vals}")

    def _set_group4(self, value: int, sync_slider: bool = True):
        v = int(value)
        if sync_slider:
            self.group4_var.set(v)
        for i in range(4):
            self.pos_scales[i].set(v)
            self.pos_set_labels[i].configure(text=str(v))
            if self.auto_send.get():
                self._safe_set(f"POS_SET({i})", v)
        self.status_var.set(f"Group 0..3 set: {v}")

    def _on_group4_change(self, value):
        self._set_group4(int(float(value)), sync_slider=False)

    def _set_pose(self, pose):
        for i, v in enumerate(pose):
            self.pos_scales[i].set(int(v))
            self.pos_set_labels[i].configure(text=str(int(v)))
            if self.auto_send.get():
                self._safe_set(f"POS_SET({i})", int(v))
        self.status_var.set(f"Pose set: {pose}")

    def _readback_once(self):
        for i in range(6):
            p = self._safe_get(f"{self.pos_read_prefix}({i})")
            f = self._safe_get(f"{self.force_read_prefix}({i})")
            self.act_pos_labels[i].configure(text=str(p))
            self.act_force_labels[i].configure(text=str(f))

        ts = time.strftime("%H:%M:%S")
        self.status_var.set(f"Readback {ts}")

    def _schedule_readback(self, initial=False):
        if initial:
            self._readback_once()
        else:
            try:
                self._readback_once()
            except Exception as exc:
                self.status_var.set(f"Readback error: {exc}")

        self.after_id = self.root.after(max(100, self.args.poll_ms), self._schedule_readback)

    def close(self):
        if self.after_id is not None:
            self.root.after_cancel(self.after_id)
        try:
            self.client.close()
        except Exception:
            pass


def parse_args():
    parser = argparse.ArgumentParser(description="RH56 GUI sliders for per-finger position tuning")
    parser.add_argument("--ip", default="192.168.123.211", help="Hand IP")
    parser.add_argument("--port", type=int, default=6000, help="Hand port")
    parser.add_argument("--speed", type=int, default=180, help="Default SPEED_SET for all channels")
    parser.add_argument("--force", type=int, default=300, help="Default FORCE_SET for all channels")
    parser.add_argument("--poll-ms", type=int, default=500, help="Readback refresh period in ms")
    return parser.parse_args()


def main():
    args = parse_args()
    client = RH56DFTP_TCP(host=args.ip, port=args.port)

    root = tk.Tk()
    app = HandSlidersApp(root, client, args)

    def _on_close():
        app.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", _on_close)
    root.mainloop()


if __name__ == "__main__":
    main()

