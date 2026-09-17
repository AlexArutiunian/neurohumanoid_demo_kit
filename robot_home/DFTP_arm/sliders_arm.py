#!/usr/bin/env python3
"""GUI sliders for RH56 hand position/force tuning with dual-hand symmetry mode."""


"""
python3 sliders_arm.py \
  --ip-left 192.168.123.210 \
  --ip-right 192.168.123.211
  
  
python3 sliders_arm.py \
  --ip-right 192.168.123.211
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

# Keep mapping explicit so we can change behavior later without touching callbacks.
MIRROR_CHANNEL_MAP = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5}


def pick_prefix(*candidates):
    for c in candidates:
        if f"{c}(0)" in ALL_REGISTER_NAMES:
            return c
    return None


def resolve_hand_targets(args):
    # Single-hand legacy mode:
    #   --ip 192.168.123.211
    if args.ip and (args.ip_left is None) and (args.ip_right is None):
        return [("hand", args.ip.strip())]

    # If only right IP is explicitly given, connect only right hand.
    # This is the important case when only 192.168.123.211 is connected.
    if args.ip_right is not None and args.ip_left is None:
        return [("right", args.ip_right.strip())]

    # If only left IP is explicitly given, connect only left hand.
    if args.ip_left is not None and args.ip_right is None:
        return [("left", args.ip_left.strip())]

    # If both are given, connect both.
    if args.ip_left is not None and args.ip_right is not None:
        left_ip = args.ip_left.strip()
        right_ip = args.ip_right.strip()

        if left_ip and right_ip and left_ip == right_ip:
            print("[WARN] --ip-left and --ip-right are identical. Running one-hand mode.")
            return [("hand", left_ip)]

        targets = []
        if left_ip:
            targets.append(("left", left_ip))
        if right_ip:
            targets.append(("right", right_ip))

        if not targets:
            raise ValueError("At least one hand IP is required")
        return targets

    # No explicit IPs: default dual-hand mode.
    return [
        ("left", "192.168.123.210"),
        ("right", "192.168.123.211"),
    ]


def connect_clients(targets, port):
    clients = []
    try:
        for hand_name, ip in targets:
            client = RH56DFTP_TCP(host=ip, port=port)
            clients.append((hand_name, ip, client))
            print(f"[INFO] Connected {hand_name} hand: {ip}:{port}")
        return clients
    except Exception:
        for _name, _ip, client in clients:
            try:
                client.close()
            except Exception:
                pass
        raise


class DualHandSlidersApp:
    def __init__(self, root, hand_clients, args):
        self.root = root
        self.args = args
        self.hand_clients = hand_clients
        self.hands = {}
        self.hand_order = [name for name, _ip, _client in hand_clients]
        self.after_id = None

        self.pos_read_prefix = pick_prefix("POS_ACT", "ANGLE_ACT", "POS_SET", "ANGLE_SET") or "POS_SET"
        self.force_read_prefix = pick_prefix("FORCE_ACT", "FORCE_SET") or "FORCE_SET"

        self.auto_send = tk.BooleanVar(value=True)
        self.symmetry_mode = tk.BooleanVar(value=False)
        self.status_var = tk.StringVar(value="Connected")

        title_ips = ", ".join([f"{name}:{ip}" for name, ip, _client in hand_clients])
        self.root.title(f"RH56 sliders  {title_ips}")
        self.root.geometry("1160x860" if len(hand_clients) > 1 else "1160x560")
        self.root.minsize(980, 520)

        self._build_ui()
        self._apply_speed_force_defaults()
        for hand_key in self.hand_order:
            self._set_pose(hand_key, OPEN_POSE, propagate=False)
        self._schedule_readback(initial=True)

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill=tk.X)

        ips_line = ", ".join([f"{name}: {ip}" for name, ip, _client in self.hand_clients])
        ttk.Label(top, text=f"Hands: {ips_line}   port={self.args.port}").pack(side=tk.LEFT)
        ttk.Checkbutton(top, text="Auto send", variable=self.auto_send).pack(side=tk.LEFT, padx=12)
        symmetry_btn = ttk.Checkbutton(top, text="Symmetry mode", variable=self.symmetry_mode)
        symmetry_btn.pack(side=tk.LEFT, padx=8)
        if len(self.hand_order) < 2:
            symmetry_btn.state(["disabled"])

        ttk.Button(top, text="Open both", command=lambda: self._set_pose_for_all(OPEN_POSE)).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Fist both", command=lambda: self._set_pose_for_all(FIST_POSE)).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Read now", command=self._readback_once).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Apply pos both", command=self._apply_all_positions_for_all).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Apply force both", command=self._apply_all_forces_for_all).pack(side=tk.LEFT, padx=4)
        ttk.Label(top, textvariable=self.status_var, foreground="#0a7d00").pack(side=tk.RIGHT)

        # Global vertical scroll for the whole UI body.
        body_host = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        body_host.pack(fill=tk.BOTH, expand=True)

        body_vsb = ttk.Scrollbar(body_host, orient=tk.VERTICAL)
        body_vsb.pack(side=tk.RIGHT, fill=tk.Y)
        body_canvas = tk.Canvas(body_host, highlightthickness=0, yscrollcommand=body_vsb.set)
        body_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        body_vsb.configure(command=body_canvas.yview)

        body = ttk.Frame(body_canvas, padding=(0, 6, 0, 0))
        body_window = body_canvas.create_window((0, 0), window=body, anchor="nw")

        body.bind("<Configure>", lambda _e: body_canvas.configure(scrollregion=body_canvas.bbox("all")))
        body_canvas.bind("<Configure>", lambda e: body_canvas.itemconfigure(body_window, width=e.width))
        self._bind_mousewheel(body_canvas, body_canvas)
        self._bind_mousewheel(body, body_canvas)

        body.grid_columnconfigure(0, weight=1)

        for row_i, (hand_name, ip, client) in enumerate(self.hand_clients):
            panel = ttk.LabelFrame(body, text=f"{hand_name.title()} hand  {ip}:{self.args.port}", padding=8)
            panel.grid(row=row_i, column=0, sticky="nsew", padx=4, pady=6)
            body.grid_rowconfigure(row_i, weight=1)
            self.hands[hand_name] = {
                "ip": ip,
                "client": client,
                "pos_scales": [],
                "pos_set_labels": [],
                "force_scales": [],
                "force_set_labels": [],
                "act_pos_labels": [],
                "act_force_labels": [],
                "status_var": tk.StringVar(value="Connected"),
                "group4_var": tk.IntVar(value=0),
                "mute_callbacks": False,
            }
            self._build_hand_panel(panel, hand_name)

    def _build_hand_panel(self, panel, hand_key):
        hand = self.hands[hand_key]

        top = ttk.Frame(panel)
        top.pack(fill=tk.X)
        ttk.Button(top, text="Open", command=lambda h=hand_key: self._set_pose(h, OPEN_POSE)).pack(side=tk.LEFT, padx=3)
        ttk.Button(top, text="Fist", command=lambda h=hand_key: self._set_pose(h, FIST_POSE)).pack(side=tk.LEFT, padx=3)
        ttk.Button(top, text="Read", command=lambda h=hand_key: self._readback_hand(h)).pack(side=tk.LEFT, padx=3)
        ttk.Button(top, text="Apply pos", command=lambda h=hand_key: self._apply_all_positions(h)).pack(side=tk.LEFT, padx=3)
        ttk.Button(top, text="Apply force", command=lambda h=hand_key: self._apply_all_forces(h)).pack(side=tk.LEFT, padx=3)
        ttk.Label(top, textvariable=hand["status_var"], foreground="#0a7d00").pack(side=tk.RIGHT)

        group = ttk.Frame(panel, padding=(0, 4, 0, 6))
        group.pack(fill=tk.X)
        ttk.Label(group, text="Group 0..3 (little/ring/middle/index):").pack(side=tk.LEFT)
        group_scale = tk.Scale(
            group,
            from_=0,
            to=2000,
            orient=tk.HORIZONTAL,
            length=320,
            showvalue=True,
            variable=hand["group4_var"],
            command=lambda v, h=hand_key: self._on_group4_change(h, v),
        )
        group_scale.pack(side=tk.LEFT, padx=8)
        ttk.Button(group, text="Open 0..3", command=lambda h=hand_key: self._set_group4(h, 0)).pack(side=tk.LEFT, padx=3)
        ttk.Button(group, text="Close 0..3", command=lambda h=hand_key: self._set_group4(h, 1800)).pack(side=tk.LEFT, padx=3)

        # Per-hand vertical scroll for the finger rows.
        rows_host = ttk.Frame(panel)
        rows_host.pack(fill=tk.BOTH, expand=True)
        rows_vsb = ttk.Scrollbar(rows_host, orient=tk.VERTICAL)
        rows_vsb.pack(side=tk.RIGHT, fill=tk.Y)
        rows_canvas = tk.Canvas(rows_host, highlightthickness=0, height=240, yscrollcommand=rows_vsb.set)
        rows_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        rows_vsb.configure(command=rows_canvas.yview)

        rows_body = ttk.Frame(rows_canvas)
        rows_window = rows_canvas.create_window((0, 0), window=rows_body, anchor="nw")
        rows_body.bind("<Configure>", lambda _e: rows_canvas.configure(scrollregion=rows_canvas.bbox("all")))
        rows_canvas.bind("<Configure>", lambda e: rows_canvas.itemconfigure(rows_window, width=e.width))
        self._bind_mousewheel(rows_canvas, rows_canvas)
        self._bind_mousewheel(rows_body, rows_canvas)

        hdr = ttk.Frame(rows_body)
        hdr.pack(fill=tk.X)
        ttk.Label(hdr, text="Finger", width=16).grid(row=0, column=0, sticky="w")
        ttk.Label(hdr, text="Pos set", width=8).grid(row=0, column=1)
        ttk.Label(hdr, text="Pos slider", width=32).grid(row=0, column=2)
        ttk.Label(hdr, text="Force set", width=8).grid(row=0, column=3)
        ttk.Label(hdr, text="Force slider", width=32).grid(row=0, column=4)
        ttk.Label(hdr, text="Act pos", width=10).grid(row=0, column=5)
        ttk.Label(hdr, text="Act force", width=10).grid(row=0, column=6)

        for i, name in enumerate(FINGER_NAMES):
            row = ttk.Frame(rows_body)
            row.pack(fill=tk.X, pady=2)

            ttk.Label(row, text=name, width=16).grid(row=0, column=0, sticky="w")

            pos_set_lbl = ttk.Label(row, text="0", width=8)
            pos_set_lbl.grid(row=0, column=1)
            hand["pos_set_labels"].append(pos_set_lbl)

            pos_scale = tk.Scale(
                row,
                from_=0,
                to=2000,
                orient=tk.HORIZONTAL,
                length=300,
                showvalue=False,
                resolution=1,
                command=lambda v, h=hand_key, idx=i: self._on_pos_slider_change(h, idx, v),
            )
            pos_scale.grid(row=0, column=2, padx=6, sticky="we")
            hand["pos_scales"].append(pos_scale)

            force_set_lbl = ttk.Label(row, text=str(self.args.force), width=8)
            force_set_lbl.grid(row=0, column=3)
            hand["force_set_labels"].append(force_set_lbl)

            force_scale = tk.Scale(
                row,
                from_=0,
                to=1000,
                orient=tk.HORIZONTAL,
                length=300,
                showvalue=False,
                resolution=1,
                command=lambda v, h=hand_key, idx=i: self._on_force_slider_change(h, idx, v),
            )
            force_scale.set(int(self.args.force))
            force_scale.grid(row=0, column=4, padx=6, sticky="we")
            hand["force_scales"].append(force_scale)

            pos_lbl = ttk.Label(row, text="-", width=10)
            pos_lbl.grid(row=0, column=5)
            hand["act_pos_labels"].append(pos_lbl)

            force_lbl = ttk.Label(row, text="-", width=10)
            force_lbl.grid(row=0, column=6)
            hand["act_force_labels"].append(force_lbl)

    @staticmethod
    def _mousewheel_steps(event):
        if hasattr(event, "num"):
            if event.num == 4:
                return -1
            if event.num == 5:
                return 1
        delta = getattr(event, "delta", 0)
        if delta == 0:
            return 0
        return int(-delta / 120)

    def _on_mousewheel(self, event, canvas):
        steps = self._mousewheel_steps(event)
        if steps != 0:
            canvas.yview_scroll(steps, "units")
            return "break"
        return None

    def _bind_mousewheel(self, widget, canvas):
        widget.bind("<MouseWheel>", lambda e, c=canvas: self._on_mousewheel(e, c), add="+")
        widget.bind("<Button-4>", lambda e, c=canvas: self._on_mousewheel(e, c), add="+")
        widget.bind("<Button-5>", lambda e, c=canvas: self._on_mousewheel(e, c), add="+")

    def _other_hand(self, hand_key):
        others = [h for h in self.hand_order if h != hand_key]
        return others[0] if others else None

    def _safe_get(self, hand_key, reg, default="-"):
        hand = self.hands[hand_key]
        try:
            return hand["client"].get(reg)
        except Exception:
            return default

    def _safe_set(self, hand_key, reg, value):
        hand = self.hands[hand_key]
        try:
            hand["client"].set(reg, int(value))
            return True
        except Exception as exc:
            msg = f"{hand_key} write error: {exc}"
            hand["status_var"].set(msg)
            self.status_var.set(msg)
            return False

    def _apply_speed_force_defaults(self):
        for hand_key in self.hand_order:
            for i in range(6):
                self._safe_set(hand_key, f"SPEED_SET({i})", self.args.speed)
                self._safe_set(hand_key, f"FORCE_SET({i})", self.args.force)

    def _set_mute(self, hand_key, value):
        self.hands[hand_key]["mute_callbacks"] = bool(value)

    def _set_pos_slider_value(self, hand_key, idx, value):
        hand = self.hands[hand_key]
        self._set_mute(hand_key, True)
        try:
            hand["pos_scales"][idx].set(int(value))
        finally:
            self._set_mute(hand_key, False)
        hand["pos_set_labels"][idx].configure(text=str(int(value)))

    def _set_force_slider_value(self, hand_key, idx, value):
        hand = self.hands[hand_key]
        self._set_mute(hand_key, True)
        try:
            hand["force_scales"][idx].set(int(value))
        finally:
            self._set_mute(hand_key, False)
        hand["force_set_labels"][idx].configure(text=str(int(value)))

    def _mirror_pos_value(self, src_hand, idx, value):
        if not self.symmetry_mode.get():
            return
        dst_hand = self._other_hand(src_hand)
        if dst_hand is None:
            return
        dst_idx = MIRROR_CHANNEL_MAP.get(idx, idx)
        self._set_pos_slider_value(dst_hand, dst_idx, value)
        if self.auto_send.get():
            self._safe_set(dst_hand, f"POS_SET({dst_idx})", value)
        self._refresh_group4_var(dst_hand)

    def _mirror_force_value(self, src_hand, idx, value):
        if not self.symmetry_mode.get():
            return
        dst_hand = self._other_hand(src_hand)
        if dst_hand is None:
            return
        dst_idx = MIRROR_CHANNEL_MAP.get(idx, idx)
        self._set_force_slider_value(dst_hand, dst_idx, value)
        if self.auto_send.get():
            self._safe_set(dst_hand, f"FORCE_SET({dst_idx})", value)

    def _on_pos_slider_change(self, hand_key, idx, value):
        hand = self.hands[hand_key]
        if hand["mute_callbacks"]:
            return
        v = int(float(value))
        hand["pos_set_labels"][idx].configure(text=str(v))
        if self.auto_send.get():
            self._safe_set(hand_key, f"POS_SET({idx})", v)
        self._refresh_group4_var(hand_key)
        self._mirror_pos_value(hand_key, idx, v)

    def _on_force_slider_change(self, hand_key, idx, value):
        hand = self.hands[hand_key]
        if hand["mute_callbacks"]:
            return
        v = int(float(value))
        hand["force_set_labels"][idx].configure(text=str(v))
        if self.auto_send.get():
            self._safe_set(hand_key, f"FORCE_SET({idx})", v)
        self._mirror_force_value(hand_key, idx, v)

    def _current_pos_targets(self, hand_key):
        return [int(s.get()) for s in self.hands[hand_key]["pos_scales"]]

    def _current_force_targets(self, hand_key):
        return [int(s.get()) for s in self.hands[hand_key]["force_scales"]]

    def _apply_all_positions(self, hand_key):
        vals = self._current_pos_targets(hand_key)
        for i, v in enumerate(vals):
            self._safe_set(hand_key, f"POS_SET({i})", v)
        self.hands[hand_key]["status_var"].set(f"Applied pos: {vals}")
        if self.symmetry_mode.get():
            dst_hand = self._other_hand(hand_key)
            if dst_hand is not None:
                mirrored = [0] * 6
                for src_idx, src_val in enumerate(vals):
                    mirrored[MIRROR_CHANNEL_MAP.get(src_idx, src_idx)] = src_val
                for i, v in enumerate(mirrored):
                    self._set_pos_slider_value(dst_hand, i, v)
                    self._safe_set(dst_hand, f"POS_SET({i})", v)
                self._refresh_group4_var(dst_hand)
                self.hands[dst_hand]["status_var"].set(f"Mirrored pos: {mirrored}")

    def _apply_all_forces(self, hand_key):
        vals = self._current_force_targets(hand_key)
        for i, v in enumerate(vals):
            self._safe_set(hand_key, f"FORCE_SET({i})", v)
        self.hands[hand_key]["status_var"].set(f"Applied force: {vals}")
        if self.symmetry_mode.get():
            dst_hand = self._other_hand(hand_key)
            if dst_hand is not None:
                mirrored = [0] * 6
                for src_idx, src_val in enumerate(vals):
                    mirrored[MIRROR_CHANNEL_MAP.get(src_idx, src_idx)] = src_val
                for i, v in enumerate(mirrored):
                    self._set_force_slider_value(dst_hand, i, v)
                    self._safe_set(dst_hand, f"FORCE_SET({i})", v)
                self.hands[dst_hand]["status_var"].set(f"Mirrored force: {mirrored}")

    def _set_group4(self, hand_key, value: int, sync_slider: bool = True):
        hand = self.hands[hand_key]
        v = int(value)
        if sync_slider:
            hand["group4_var"].set(v)
        for i in range(4):
            self._set_pos_slider_value(hand_key, i, v)
            if self.auto_send.get():
                self._safe_set(hand_key, f"POS_SET({i})", v)
        hand["status_var"].set(f"Group 0..3 set: {v}")
        if self.symmetry_mode.get():
            dst_hand = self._other_hand(hand_key)
            if dst_hand is not None:
                for i in range(4):
                    dst_idx = MIRROR_CHANNEL_MAP.get(i, i)
                    self._set_pos_slider_value(dst_hand, dst_idx, v)
                    if self.auto_send.get():
                        self._safe_set(dst_hand, f"POS_SET({dst_idx})", v)
                self._refresh_group4_var(dst_hand)
                self.hands[dst_hand]["status_var"].set(f"Mirrored group 0..3: {v}")

    def _on_group4_change(self, hand_key, value):
        self._set_group4(hand_key, int(float(value)), sync_slider=False)

    def _refresh_group4_var(self, hand_key):
        hand = self.hands[hand_key]
        vals = [int(hand["pos_scales"][i].get()) for i in range(4)]
        if len(set(vals)) == 1:
            hand["group4_var"].set(vals[0])

    def _set_pose(self, hand_key, pose, propagate=True):
        for i, v in enumerate(pose):
            self._set_pos_slider_value(hand_key, i, int(v))
            if self.auto_send.get():
                self._safe_set(hand_key, f"POS_SET({i})", int(v))
        self._refresh_group4_var(hand_key)
        self.hands[hand_key]["status_var"].set(f"Pose set: {list(map(int, pose))}")

        if propagate and self.symmetry_mode.get():
            dst_hand = self._other_hand(hand_key)
            if dst_hand is not None:
                mirrored = [0] * 6
                for src_idx, src_val in enumerate(pose):
                    mirrored[MIRROR_CHANNEL_MAP.get(src_idx, src_idx)] = int(src_val)
                self._set_pose(dst_hand, mirrored, propagate=False)

    def _set_pose_for_all(self, pose):
        for hand_key in self.hand_order:
            self._set_pose(hand_key, pose, propagate=False)

    def _apply_all_positions_for_all(self):
        sym_prev = bool(self.symmetry_mode.get())
        self.symmetry_mode.set(False)
        try:
            for hand_key in self.hand_order:
                self._apply_all_positions(hand_key)
        finally:
            self.symmetry_mode.set(sym_prev)

    def _apply_all_forces_for_all(self):
        sym_prev = bool(self.symmetry_mode.get())
        self.symmetry_mode.set(False)
        try:
            for hand_key in self.hand_order:
                self._apply_all_forces(hand_key)
        finally:
            self.symmetry_mode.set(sym_prev)

    def _readback_hand(self, hand_key):
        hand = self.hands[hand_key]
        for i in range(6):
            p = self._safe_get(hand_key, f"{self.pos_read_prefix}({i})")
            f = self._safe_get(hand_key, f"{self.force_read_prefix}({i})")
            hand["act_pos_labels"][i].configure(text=str(p))
            hand["act_force_labels"][i].configure(text=str(f))
        ts = time.strftime("%H:%M:%S")
        hand["status_var"].set(f"Readback {ts}")

    def _readback_once(self):
        for hand_key in self.hand_order:
            self._readback_hand(hand_key)
        ts = time.strftime("%H:%M:%S")
        self.status_var.set(f"Readback all {ts}")

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
        for hand_key in self.hand_order:
            try:
                self.hands[hand_key]["client"].close()
            except Exception:
                pass


def parse_args():
    parser = argparse.ArgumentParser(description="RH56 dual-hand GUI sliders for per-finger position tuning")
    parser.add_argument(
        "--ip",
        default=None,
        help="Legacy single-hand IP. If set and --ip-left/--ip-right are omitted, runs one-hand mode.",
    )
    parser.add_argument("--ip-left", default=None, help="Left hand IP (default: 192.168.123.210)")
    parser.add_argument("--ip-right", default=None, help="Right hand IP (default: 192.168.123.211)")
    parser.add_argument("--port", type=int, default=6000, help="Hand port")
    parser.add_argument("--speed", type=int, default=180, help="Default SPEED_SET for all channels")
    parser.add_argument("--force", type=int, default=300, help="Default FORCE_SET for all channels")
    parser.add_argument("--poll-ms", type=int, default=500, help="Readback refresh period in ms")
    return parser.parse_args()


def main():
    args = parse_args()
    targets = resolve_hand_targets(args)
    hand_clients = connect_clients(targets, args.port)

    root = tk.Tk()
    app = DualHandSlidersApp(root, hand_clients, args)

    def _on_close():
        app.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", _on_close)
    root.mainloop()


if __name__ == "__main__":
    main()

