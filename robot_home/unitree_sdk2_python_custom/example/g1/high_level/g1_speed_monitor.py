#!/usr/bin/env python3
import math
import sys
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_


def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    topic = sys.argv[2] if len(sys.argv) > 2 else "rt/sportmodestate"
    print_period_s = 0.1
    eps = 1e-3
    yaw_min_abs = 0.05

    ChannelFactoryInitialize(0, iface)

    print(f"Interface: {iface}")
    print(f"Topic:Ф {topic}")
    print("Reading speed from sport state. Press Ctrl+C to stop.")
    last_print_t = 0.0

    def on_state(msg: SportModeState_):
        nonlocal last_print_t
        now = time.monotonic()
        if now - last_print_t < print_period_s:
            return

        vx = float(msg.velocity[0])
        vy = float(msg.velocity[1])
        vyaw = float(msg.yaw_speed)
        if abs(vyaw) < yaw_min_abs:
            return
        if abs(vx) < eps and abs(vy) < eps and abs(vyaw) < eps:
            return

        v = math.sqrt(vx * vx + vy * vy)
        print(f"vx={vx:+.3f}  vy={vy:+.3f}  |v|={v:.3f} m/s  yaw={vyaw:+.3f} rad/s")
        last_print_t = now

    sub = ChannelSubscriber(topic, SportModeState_)
    sub.Init(on_state, 50)

    try:
        while True:
            time.sleep(1)
    finally:
        sub.Close()


if __name__ == "__main__":
    main()

