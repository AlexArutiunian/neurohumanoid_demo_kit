import time
import sys

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC


class J:
    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14

    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21

    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28

    NotUsedJoint0 = 29


WAIST = [J.WaistYaw, J.WaistRoll, J.WaistPitch]
ARMS = [
    J.LeftShoulderPitch, J.LeftShoulderRoll, J.LeftShoulderYaw,
    J.LeftElbow, J.LeftWristRoll, J.LeftWristPitch, J.LeftWristYaw,
    J.RightShoulderPitch, J.RightShoulderRoll, J.RightShoulderYaw,
    J.RightElbow, J.RightWristRoll, J.RightWristPitch, J.RightWristYaw,
]

KP_WAIST = 800.0
KD_WAIST = 15.0
DT = 0.02

TOPIC_STATE = "rt/lowstate"
TOPIC_CMD = "rt/arm_sdk"   # для motion/arm_sdk режима


def init_dds():
    # Если запустили: python hold_waist.py eth0
    # тогда использует указанный интерфейс.
    if len(sys.argv) >= 2:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)


def wait_lowstate(sub, timeout=10.0):
    print("[INFO] Жду lowstate...")
    t0 = time.time()

    while time.time() - t0 < timeout:
        msg = sub.Read()
        if msg is not None:
            return msg
        time.sleep(0.01)

    print("[ERROR] Не получил lowstate.")
    print("Проверьте, что робот включен, сеть настроена и выбран правильный режим.")
    sys.exit(1)


def make_cmd(crc, mode_machine, target_q, arm_q, weight):
    cmd = unitree_hg_msg_dds__LowCmd_()

    cmd.mode_pr = 0
    cmd.mode_machine = mode_machine

    # Талия: держим текущую позицию жестко.
    for j in WAIST:
        cmd.motor_cmd[j].mode = 1
        cmd.motor_cmd[j].q = target_q[j]
        cmd.motor_cmd[j].dq = 0.0
        cmd.motor_cmd[j].kp = KP_WAIST
        cmd.motor_cmd[j].kd = KD_WAIST
        cmd.motor_cmd[j].tau = 0.0

    # Руки: q=current, но kp=0/kd=0/tau=0.
    for j in ARMS:
        cmd.motor_cmd[j].mode = 1
        cmd.motor_cmd[j].q = arm_q[j]
        cmd.motor_cmd[j].dq = 0.0
        cmd.motor_cmd[j].kp = 0.0
        cmd.motor_cmd[j].kd = 0.0
        cmd.motor_cmd[j].tau = 0.0

    # Вес arm_sdk: 1.0 = управление через rt/arm_sdk, 0.0 = отпустить.
    cmd.motor_cmd[J.NotUsedJoint0].mode = 1
    cmd.motor_cmd[J.NotUsedJoint0].q = weight
    cmd.motor_cmd[J.NotUsedJoint0].dq = 0.0
    cmd.motor_cmd[J.NotUsedJoint0].kp = 0.0
    cmd.motor_cmd[J.NotUsedJoint0].kd = 0.0
    cmd.motor_cmd[J.NotUsedJoint0].tau = 0.0

    cmd.crc = crc.Crc(cmd)
    return cmd


def main():
    init_dds()

    sub = ChannelSubscriber(TOPIC_STATE, LowState_)
    sub.Init()

    pub = ChannelPublisher(TOPIC_CMD, LowCmd_)
    pub.Init()

    crc = CRC()

    state = wait_lowstate(sub)
    mode_machine = state.mode_machine

    target_q = {}
    arm_q = {}

    for j in WAIST:
        target_q[j] = state.motor_state[j].q

    for j in ARMS:
        arm_q[j] = state.motor_state[j].q

    print("[OK] Зафиксировал текущую талию:")
    print(f"  WaistYaw   = {target_q[J.WaistYaw]:+.4f} rad")
    print(f"  WaistRoll  = {target_q[J.WaistRoll]:+.4f} rad")
    print(f"  WaistPitch = {target_q[J.WaistPitch]:+.4f} rad")
    print(f"[INFO] Держу талию kp={KP_WAIST}, kd={KD_WAIST}")
    print("[INFO] Руки: q=current, kp=0, kd=0, tau=0")
    print("[INFO] Ctrl+C — выйти и отпустить arm_sdk")

    try:
        # Плавно включаем вес arm_sdk.
        for i in range(1, 51):
            weight = i / 50.0
            cmd = make_cmd(crc, mode_machine, target_q, arm_q, weight)
            pub.Write(cmd)
            time.sleep(DT)

        while True:
            cmd = make_cmd(crc, mode_machine, target_q, arm_q, 1.0)
            pub.Write(cmd)
            time.sleep(DT)

    except KeyboardInterrupt:
        print("\n[INFO] Выход: плавно отпускаю arm_sdk...")

    finally:
        # Плавно отпускаем управление.
        for i in range(50, -1, -1):
            weight = i / 50.0
            cmd = make_cmd(crc, mode_machine, target_q, arm_q, weight)
            pub.Write(cmd)
            time.sleep(DT)

        print("[OK] Готово.")


if __name__ == "__main__":
    main()
