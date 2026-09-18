# Neurohumanoid Demo Kit — Unitree G1 JSON Control

Transfer kit for moving the working Unitree G1 upper-body JSON control setup to another robot.

This branch contains the cleaned robot-side files for:

- Unitree SDK2 Python custom tree used by the demo;
- joystick-triggered JSON motion playback;
- actual motion recording through ROS 2 `/joint_states` and `/tf`;
- Unitree G1 URDF with Inspire hand;
- RH56DFTP / Inspire hand example scripts and tactile toolkit notes.

## Ubuntu / ROS 2 compatibility

The JSON arm playback itself is mostly independent of Ubuntu 20 vs Ubuntu 22 as long as `unitree_sdk2py` works and the correct network interface is used.

The ROS recording path depends on the installed ROS 2 distro:

- Ubuntu 22 usually uses ROS 2 Humble: `/opt/ros/humble/setup.bash`;
- Ubuntu 20 usually uses ROS 2 Foxy: `/opt/ros/foxy/setup.bash`.

This branch auto-detects Humble or Foxy in:

- `robot_home/unitree_sdk2_python_custom/example/g1/high_level/joystick_launch_json_record.py` when it starts `record_actual_motion_ros.py`;
- the generated `~/g1_json_demo_env.sh` created by `scripts/install_to_robot.sh`.

If neither Humble nor Foxy exists, JSON playback can still run with `--no-record`, but ROS-based recording will not work until ROS 2 is installed/configured.

## Repository layout

```text
robot_home/
├── unitree_sdk2_python_custom/
│   ├── unitree_sdk2py/
│   └── example/g1/high_level/
│       ├── joystick_launch_json_record.py
│       ├── record_actual_motion_ros.py
│       ├── g1_json_upper.py
│       ├── sliders_arm.py
│       └── dataset_100/
├── joints_xyz/
│   └── lowstate_to_jointstates_ros.py
├── DFTP_arm/
│   ├── sliders_arm.py
│   ├── hand_remote_poses.py
│   ├── tactile-rh56dftp/
│   └── inspirehand_Ros2/
└── g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf
```

## What was intentionally not included

Heavy/generated files are intentionally excluded from the transfer kit:

- Python virtual environments: `.venv/`, `venv/`;
- ROS/colcon build outputs: `build/`, `install/`, `log/`;
- motion logs and recorded datasets: `actual_motion_logs/`, `data_steer/`, `data_real_states/`, `data_real_keyframes/`;
- RGB-D datasets: `dataset_rgbd/`;
- binary archives and temporary outputs: `*.zip`, `*.bag`, `*.db3`, `*.log`, `*.pyc`.

The transfer kit is meant to preserve the code and small JSON motions, not old recordings or local environments.

## External dependency for Inspire/RH56DFTP hand

The `DFTP_arm` folder contains example scripts and tactile-toolkit files, but the Python packages below may already be installed in the robot environment and are not guaranteed to be vendored in this repo:

- `RH56DFTP`
- `Register`

Check them on the robot:

```bash
python3 - <<'PY'
import RH56DFTP.RH56DFTP_TCP as tcp
import Register.RegisterKey.ftp_registers_keys as regs
print("OK RH56DFTP:", tcp.__file__)
print("OK Register:", regs.__file__)
PY
```

If this fails, install/copy the Inspire RH56DFTP Python API packages separately before using hand-control scripts.

## Install to a new robot

On the new Unitree G1 robot:

```bash
cd ~
git clone -b json-ctrl https://github.com/AlexArutiunian/neurohumanoid_demo_kit.git
cd neurohumanoid_demo_kit
bash scripts/install_to_robot.sh
```

By default the install script copies files into `/home/unitree`. To install into another home directory:

```bash
TARGET_HOME=/home/unitree bash scripts/install_to_robot.sh
```

The installer also creates:

```bash
~/g1_json_demo_env.sh
```

It is optional for pure JSON playback from the `high_level` folder, but useful for ROS setup and imports:

```bash
source ~/g1_json_demo_env.sh
```

## Run the JSON joystick demo

### Terminal 1 — publish `/joint_states`

```bash
source ~/g1_json_demo_env.sh
cd ~/joints_xyz
python3 lowstate_to_jointstates_ros.py
```

### Terminal 2 — robot state publisher

```bash
source ~/g1_json_demo_env.sh
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p robot_description:="$(cat ~/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf)"
```

### Terminal 3 — joystick launcher

```bash
cd ~/unitree_sdk2_python_custom/example/g1/high_level

python3 joystick_launch_json_record.py eth0 \
  --dataset-dir dataset_100 \
  --json-a wave_right.json \
  --json-b hold_box.json \
  --json-down 0.json \
  --duration-scale-a 1.0 \
  --duration-scale-b 1.0 \
  --duration-scale-down 1.0 \
  --kp 60 \
  --kd 1.5 \
  --kp-waist 250 \
  --kd-waist 6 \
  --record-out-dir ./actual_motion_logs \
  --record-rate-hz 50 \
  --record-target-frame pelvis
```

If ROS recording is not needed, the same launcher can run without recording:

```bash
cd ~/unitree_sdk2_python_custom/example/g1/high_level

python3 joystick_launch_json_record.py eth0 \
  --dataset-dir dataset_100 \
  --json-a wave_right.json \
  --json-b hold_box.json \
  --json-down 0.json \
  --no-record
```

Joystick mapping in `joystick_launch_json_record.py`:

- `A` — run JSON selected by `--json-a`;
- `B` — run JSON selected by `--json-b`;
- `Down` — run JSON selected by `--json-down`;
- `Up` — skip the current empty hold frame in the JSON player;
- `X` — send right-hand open pose;
- `Y` — send right-hand fist pose;
- `Select + Start` — exit launcher.

## Quick health checks

Check core files after install:

```bash
for f in \
  ~/unitree_sdk2_python_custom/example/g1/high_level/joystick_launch_json_record.py \
  ~/unitree_sdk2_python_custom/example/g1/high_level/record_actual_motion_ros.py \
  ~/unitree_sdk2_python_custom/example/g1/high_level/g1_json_upper.py \
  ~/unitree_sdk2_python_custom/example/g1/high_level/dataset_100/wave_right.json \
  ~/unitree_sdk2_python_custom/example/g1/high_level/dataset_100/hold_box.json \
  ~/joints_xyz/lowstate_to_jointstates_ros.py \
  ~/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf \
  ~/DFTP_arm/sliders_arm.py \
  ~/DFTP_arm/hand_remote_poses.py
 do
  [ -e "$f" ] && echo "OK   $f" || echo "MISS $f"
done
```

Check selected ROS 2 environment:

```bash
source ~/g1_json_demo_env.sh
echo "ROS_DISTRO=${G1_DEMO_ROS_DISTRO:-none}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-none}"
```

Check Unitree SDK import:

```bash
cd ~/unitree_sdk2_python_custom
python3 - <<'PY'
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
print("OK unitree_sdk2py imports")
PY
```

Check RH56DFTP hand package import:

```bash
python3 - <<'PY'
import RH56DFTP.RH56DFTP_TCP as tcp
import Register.RegisterKey.ftp_registers_keys as regs
print("OK RH56DFTP:", tcp.__file__)
print("OK Register:", regs.__file__)
PY
```

## Safety notes

Do not run old direct low-level arm control scripts in parallel with `joystick_launch_json_record.py` or `g1_json_upper.py`.

Do not run `sliders_arm.py` at the same time as the joystick launcher if both try to control the same RH56DFTP hand over TCP.

For a new robot, first test with short/simple JSON motions and keep emergency stop access ready.
