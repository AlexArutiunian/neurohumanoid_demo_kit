cd ~/unitree_sdk2_python
source .venv/bin/activate
export CYCLONEDDS_HOME=~/cyclonedds/install

python - <<'PY'
import time, sys, signal
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
PY  time.sleep(0.02)c(cmd)d = kd_waistst_q[j])tate[j].qые.[j].q)istYaw,
[INFO] Жду lowstate...
[OK] Зафиксировал текущую талию:
  WaistYaw   = -0.0019 rad
  WaistRoll  = -0.0008 rad
  WaistPitch = -0.0023 rad
[INFO] Держу талию kp=800.0, kd=15.0
[INFO] Руки: q=current, kp=0, kd=0, tau=0
[INFO] Ctrl+C — выйти и отпустить arm_sdk

