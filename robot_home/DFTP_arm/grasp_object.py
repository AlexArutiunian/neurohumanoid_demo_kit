#!/usr/bin/env python3
"""
Force-limited grasp script for RH56 DFTP hand.

Workflow:
1) Open hand.
2) Set per-finger force limits.
3) Close hand incrementally.
4) Stop moving each finger when measured force reaches target.
5) Hold and optionally release.

python grasp_object.py --ip 192.168.123.211 --menu --manual-release --hold-sec 180
python grasp_object.py --ip 192.168.123.210 --ip2 192.168.123.211 --menu
python grasp_object.py --ips 192.168.123.210,192.168.123.211 --menu

"""

import argparse
import select
import sys
import time

from RH56DFTP.RH56DFTP_TCP import RH56DFTP_TCP

GRASP_PRESETS = {
    # Текущий "обычный" хват всеми пальцами
    "power": {
        "close": [1800, 1800, 1800, 1800, 1800, 0],
        "force": [250, 300, 300, 350, 400, 180],
        "speed": [120, 120, 120, 120, 120, 80],
    },
    # OK-пинч (указательный + большой), остальные открыты
    "ok_pinch": {
        "close": [0, 0, 0, 1200, 500, 2000],
        "force": [0, 0, 0, 180, 220, 0],
        "speed": [0, 0, 0, 45, 35, 15],
    },
    # Широкий OK-пинч
    "ok_wide": {
        "close": [0, 0, 0, 1000, 350, 2000],
        "force": [0, 0, 0, 160, 200, 0],
        "speed": [0, 0, 0, 40, 30, 12],
    },
    # Узкий OK-пинч
    "ok_narrow": {
        "close": [0, 0, 0, 1400, 700, 2000],
        "force": [0, 0, 0, 190, 240, 0],
        "speed": [0, 0, 0, 45, 35, 15],
    },
    # Тест: только последний сустав большого пальца (канал 5) в 2000
    "thumb_last_2000": {
        "close": [0, 0, 0, 0, 0, 2000],
        "force": [0, 0, 0, 0, 0, 0],
        "speed": [0, 0, 0, 0, 0, 25],
    },
}


def parse_force_list(raw: str):
    values = [int(x.strip()) for x in raw.split(",") if x.strip()]
    if len(values) != 6:
        raise ValueError("force list must contain exactly 6 integers")
    return values


def parse_pos_list(raw: str):
    values = [int(x.strip()) for x in raw.split(",") if x.strip()]
    if len(values) != 6:
        raise ValueError("position list must contain exactly 6 integers")
    return values


def parse_ip_list(raw: str):
    values = [x.strip() for x in raw.split(",") if x.strip()]
    if not values:
        raise ValueError("IP list must not be empty")
    unique = []
    for ip in values:
        if ip not in unique:
            unique.append(ip)
    return unique


def resolve_hand_ips(args):
    if args.ips:
        return parse_ip_list(args.ips)

    values = []
    if args.ip.strip():
        values.append(args.ip.strip())
    if args.ip2.strip():
        values.append(args.ip2.strip())
    if not values:
        raise ValueError("At least one hand IP is required")

    unique = []
    for ip in values:
        if ip not in unique:
            unique.append(ip)
    return unique


def connect_clients(ips, port):
    clients = []
    try:
        for ip in ips:
            client = RH56DFTP_TCP(host=ip, port=port)
            clients.append((ip, client))
            print(f"[INFO] Connected to {ip}:{port}")
        return clients
    except Exception:
        for _, client in clients:
            try:
                client.close()
            except Exception:
                pass
        raise


def set_all_positions(client, positions):
    for i, pos in enumerate(positions):
        client.set(f"POS_SET({i})", int(pos))


def set_force_limits(client, force_limits):
    for i, value in enumerate(force_limits):
        client.set(f"FORCE_SET({i})", int(value))


def set_speed_limits(client, speed_limits):
    for i, value in enumerate(speed_limits):
        client.set(f"SPEED_SET({i})", int(value))


def read_forces(client):
    data = []
    for i in range(6):
        data.append(int(client.get(f"FORCE_ACT({i})")))
    return data


def step_towards(cur, target, step):
    if cur < target:
        return min(target, cur + step)
    if cur > target:
        return max(target, cur - step)
    return cur


def resolve_targets(args, preset_name):
    preset = GRASP_PRESETS[preset_name]
    force_limits = parse_force_list(args.force) if args.force else list(preset["force"])
    speed_limits = parse_force_list(args.speed) if args.speed else list(preset["speed"])
    close_targets = (
        parse_pos_list(args.close)
        if args.close
        else list(preset.get("close", [args.close_pos] * 6))
    )
    return force_limits, speed_limits, close_targets


def execute_grasp(
    clients,
    args,
    preset_name,
    *,
    open_before=True,
    finalize=True,
    start_pos=None,
):
    force_limits, speed_limits, close_targets = resolve_targets(args, preset_name)
    print(f"[INFO] Grasp preset: {preset_name}")

    hand_count = len(clients)
    if hand_count == 0:
        raise RuntimeError("No connected hands")

    # 1) Open (optional, for menu transitions we may keep current hold)
    if start_pos is None:
        current_pos = [[args.open_pos] * 6 for _ in range(hand_count)]
    else:
        if len(start_pos) != hand_count:
            raise ValueError("start_pos must contain positions for each connected hand")
        current_pos = [list(pos) for pos in start_pos]

    frozen = [[False] * 6 for _ in range(hand_count)]
    moved_once = [[False] * 6 for _ in range(hand_count)]
    if open_before:
        for hand_idx, (_, client) in enumerate(clients):
            set_all_positions(client, current_pos[hand_idx])
        time.sleep(0.5)

    # 2) Set force limits
    for ip, client in clients:
        set_speed_limits(client, speed_limits)
        print(f"[INFO][{ip}] Speed limits set: {speed_limits}")
    time.sleep(0.2)

    for ip, client in clients:
        set_force_limits(client, force_limits)
        print(f"[INFO][{ip}] Force limits set: {force_limits}")
    time.sleep(0.2)
    baseline_forces = []
    for ip, client in clients:
        base = read_forces(client)
        baseline_forces.append(base)
        print(f"[INFO][{ip}] Baseline force: {base}")

    # 3-4) Move gradually toward target and freeze only on force-limited closing
    while True:
        all_hands_done = True
        for hand_idx, (ip, client) in enumerate(clients):
            done = True
            forces = read_forces(client)
            delta_forces = [max(0, forces[i] - baseline_forces[hand_idx][i]) for i in range(6)]

            for i in range(6):
                if frozen[hand_idx][i]:
                    continue

                target = close_targets[i]
                cur = current_pos[hand_idx][i]
                closing = target > cur

                # Freeze only on closing direction; on opening direction we always allow motion.
                if (
                    closing
                    and force_limits[i] > 0
                    and moved_once[hand_idx][i]
                    and delta_forces[i] >= max(0, force_limits[i] - args.margin)
                ):
                    frozen[hand_idx][i] = True
                    continue

                next_pos = step_towards(cur, target, args.step)
                if next_pos != cur:
                    current_pos[hand_idx][i] = next_pos
                    moved_once[hand_idx][i] = True
                    done = False

            set_all_positions(client, current_pos[hand_idx])
            print(
                f"[INFO][{ip}] pos={current_pos[hand_idx]} force={forces} "
                f"delta={delta_forces} frozen={frozen[hand_idx]}"
            )
            if not (all(frozen[hand_idx]) or done):
                all_hands_done = False

        time.sleep(args.sleep)

        if all_hands_done:
            break

    # Optional phase: only non-contact fingers continue closing to try touching the object.
    if args.search_contact:
        while True:
            changed_any = False
            for hand_idx, (ip, client) in enumerate(clients):
                changed = False
                forces = read_forces(client)
                delta_forces = [max(0, forces[i] - baseline_forces[hand_idx][i]) for i in range(6)]
                for i in range(6):
                    if delta_forces[i] >= args.contact_threshold:
                        continue
                    if current_pos[hand_idx][i] < args.max_close:
                        current_pos[hand_idx][i] = min(args.max_close, current_pos[hand_idx][i] + args.step)
                        changed = True
                if changed:
                    set_all_positions(client, current_pos[hand_idx])
                    print(
                        f"[INFO][{ip}] search pos={current_pos[hand_idx]} "
                        f"force={forces} delta={delta_forces}"
                    )
                    changed_any = True
            if not changed_any:
                break
            time.sleep(args.sleep)

    if not finalize:
        print("[INFO] Grasp applied. Use menu to switch grasp or 'R' to release.")
        return current_pos

    if args.manual_release:
        if args.hold_sec > 0:
            print("")
            print("=== HOLD MODE ===")
            print(
                f"Press Enter to release now (auto-release in {args.hold_sec:.2f}s)"
            )
            print("> ", end="", flush=True)
            ready, _, _ = select.select([sys.stdin], [], [], args.hold_sec)
            if ready:
                sys.stdin.readline()
                print("[INFO] Manual release requested")
            else:
                print("[INFO] Auto-release timeout reached")
        else:
            print("")
            print("=== HOLD MODE ===")
            print("Press Enter to release")
            input("> ")
            print("[INFO] Manual release requested")
    else:
        print(f"[INFO] Hold for {args.hold_sec:.2f}s")
        time.sleep(args.hold_sec)

    # 5) Release
    if not args.no_release:
        for _, client in clients:
            set_all_positions(client, [args.open_pos] * 6)
        print("[INFO] Released")
    return [[args.open_pos] * 6 for _ in range(hand_count)]


def print_menu():
    print("\n" + "=" * 50)
    print("GRASP MENU")
    print("=" * 50)
    print("  1. power")
    print("  2. ok_pinch")
    print("  3. ok_wide")
    print("  4. ok_narrow")
    print("  5. thumb_last_2000")
    print("  R. Open hand / release")
    print("  X. Exit")
    print("-" * 50)


def main():
    parser = argparse.ArgumentParser(description="Object grasp with force limits")
    parser.add_argument("--ip", default="192.168.123.211", help="Primary hand IP")
    parser.add_argument("--ip2", default="", help="Second hand IP (optional)")
    parser.add_argument(
        "--ips",
        default="",
        help="Comma-separated list of hand IPs. Overrides --ip and --ip2.",
    )
    parser.add_argument("--port", type=int, default=6000, help="Hand port")
    parser.add_argument(
        "--grasp-preset",
        choices=list(GRASP_PRESETS.keys()),
        default="power",
        help="Predefined grasp profile",
    )
    parser.add_argument("--menu", action="store_true", help="Interactive mode like gestures menu")
    parser.add_argument(
        "--force",
        default="",
        help="Per-finger force limits, 6 values: f0,f1,f2,f3,f4,f5 (0..1000). Overrides preset.",
    )
    parser.add_argument(
        "--speed",
        default="",
        help="Per-finger speed limits, 6 values: s0,s1,s2,s3,s4,s5 (0..1000). Overrides preset.",
    )
    parser.add_argument("--open-pos", type=int, default=0, help="Open position")
    parser.add_argument("--close-pos", type=int, default=1800, help="Target close position")
    parser.add_argument(
        "--close",
        default="",
        help="Per-finger close targets (6 values). Overrides preset/--close-pos.",
    )
    parser.add_argument("--step", type=int, default=120, help="Close step per loop")
    parser.add_argument("--sleep", type=float, default=0.08, help="Delay between loops")
    parser.add_argument("--margin", type=int, default=20, help="Force stop margin")
    parser.add_argument(
        "--contact-threshold",
        type=int,
        default=25,
        help="Minimum force delta to treat finger as contacted",
    )
    parser.add_argument(
        "--search-contact",
        action="store_true",
        help="After main grasp, keep closing only non-contact fingers up to max",
    )
    parser.add_argument(
        "--max-close",
        type=int,
        default=2000,
        help="Upper close bound used by --search-contact",
    )
    parser.add_argument("--hold-sec", type=float, default=30.0, help="Hold time / auto-release timeout")
    parser.add_argument(
        "--manual-release",
        action="store_true",
        help="Release on Enter key; if --hold-sec > 0, auto-release by timeout",
    )
    parser.add_argument(
        "--no-release",
        action="store_true",
        help="Do not open hand after hold",
    )
    args = parser.parse_args()

    hand_ips = resolve_hand_ips(args)
    clients = connect_clients(hand_ips, args.port)
    print(f"[INFO] Hands in control loop: {', '.join(hand_ips)}")

    try:
        if not args.menu:
            execute_grasp(clients, args, args.grasp_preset)
            return

        choice_to_preset = {
            "1": "power",
            "2": "ok_pinch",
            "3": "ok_wide",
            "4": "ok_narrow",
            "5": "thumb_last_2000",
        }
        has_active_grasp = False
        menu_pos = [[args.open_pos] * 6 for _ in clients]
        active_preset = None
        while True:
            print_menu()
            if active_preset is None:
                print("[INFO] active: open")
            else:
                print(f"[INFO] active: {active_preset}")
            choice = input("\nChoose grasp or command: ").strip().upper()
            if choice == "X":
                print("[INFO] Exit menu")
                break
            if choice == "R":
                for _, client in clients:
                    set_all_positions(client, [args.open_pos] * 6)
                print("[INFO] Hands opened")
                has_active_grasp = False
                menu_pos = [[args.open_pos] * 6 for _ in clients]
                active_preset = None
                continue
            preset = choice_to_preset.get(choice)
            if preset is None:
                print("[INFO] Unknown choice")
                continue
            result_pos = execute_grasp(
                clients,
                args,
                preset,
                open_before=not has_active_grasp,
                finalize=False,
                start_pos=menu_pos,
            )
            if result_pos is not None:
                menu_pos = [list(pos) for pos in result_pos]
            has_active_grasp = True
            active_preset = preset
    finally:
        for ip, client in clients:
            try:
                client.close()
                print(f"[INFO] Connection closed: {ip}:{args.port}")
            except Exception as exc:
                print(f"[WARN] Failed to close {ip}:{args.port}: {exc}")


if __name__ == "__main__":
    main()
