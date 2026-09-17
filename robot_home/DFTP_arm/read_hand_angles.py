#!/usr/bin/env python3
import argparse
import logging
import time

from RH56DFTP.RH56DFTP_TCP import RH56DFTP_TCP
from Register.RegisterKey.ftp_registers_keys import ALL_REGISTER_NAMES


def read_group(client, prefix):
    vals = []
    for i in range(6):
        vals.append(client.get(f"{prefix}({i})"))
    return vals


def fmt(values):
    return " ".join(f"{v:>5}" for v in values)


def pick_prefix(*candidates):
    for c in candidates:
        if f"{c}(0)" in ALL_REGISTER_NAMES:
            return c
    return None


def _to_float_list(values):
    out = []
    for v in values:
        try:
            out.append(float(v))
        except Exception:
            out.append(0.0)
    return out


def changed_with_eps(cur, prev, eps):
    if prev is None:
        return True
    if len(cur) != len(prev):
        return True
    for a, b in zip(cur, prev):
        if abs(a - b) > eps:
            return True
    return False


def main():
    parser = argparse.ArgumentParser(description="Read current hand angles/positions")
    parser.add_argument("--ip", default="192.168.123.211", help="Hand IP")
    parser.add_argument("--port", type=int, default=6000, help="Hand port")
    parser.add_argument("--hz", type=float, default=2.0, help="Polling rate")
    parser.add_argument("--once", action="store_true", help="Read once and exit")
    parser.add_argument(
        "--print-all",
        action="store_true",
        help="Print every poll (by default prints only on changes)",
    )
    parser.add_argument(
        "--eps",
        type=float,
        default=3.0,
        help="Minimum absolute change to print update (ignored with --print-all)",
    )
    parser.add_argument(
        "--sdk-log-level",
        default="WARNING",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
        help="Log level for RH56DFTP/pymodbus internals",
    )
    args = parser.parse_args()

    # Silence noisy internal SDK logs by default.
    lvl = getattr(logging, args.sdk_log_level.upper(), logging.WARNING)
    logging.getLogger("RH56DFTP").setLevel(lvl)
    logging.getLogger("pymodbus").setLevel(lvl)

    period = 1.0 / max(0.1, args.hz)
    client = RH56DFTP_TCP(host=args.ip, port=args.port)
    print(f"[INFO] Connected to {args.ip}:{args.port}")
    print("[INFO] channels: 0..5 (little->thumb bend->thumb rotate)")

    pos_prefix = pick_prefix("POS_ACT", "ANGLE_ACT", "POS_SET", "ANGLE_SET")
    force_prefix = pick_prefix("FORCE_ACT", "FORCE_SET")

    if pos_prefix is None:
        raise RuntimeError("No position/angle register family found in this SDK build")
    if force_prefix is None:
        raise RuntimeError("No force register family found in this SDK build")

    print(f"[INFO] using position register family: {pos_prefix}(i)")
    print(f"[INFO] using force register family: {force_prefix}(i)")
    print("[INFO] print mode: every poll" if args.print_all else "[INFO] print mode: on change")
    if pos_prefix in ("POS_SET", "ANGLE_SET"):
        print(
            "[WARN] This firmware/SDK does not expose actual finger position registers here. "
            "You are seeing command setpoints, not live joint feedback."
        )

    try:
        last_pos = None
        last_force = None
        while True:
            ts = time.strftime("%H:%M:%S")
            pos_act = read_group(client, pos_prefix)
            force_act = read_group(client, force_prefix)

            pos_num = _to_float_list(pos_act)
            force_num = _to_float_list(force_act)
            changed = changed_with_eps(pos_num, last_pos, args.eps) or changed_with_eps(
                force_num, last_force, args.eps
            )
            if args.print_all or changed or args.once:
                print(
                    f"[{ts}] {pos_prefix.lower()}:{fmt(pos_act)}"
                    f" {force_prefix.lower()}:{fmt(force_act)}"
                )
                last_pos = pos_num
                last_force = force_num

            if args.once:
                break
            time.sleep(period)
    finally:
        client.close()
        print("[INFO] Connection closed")


if __name__ == "__main__":
    main()
