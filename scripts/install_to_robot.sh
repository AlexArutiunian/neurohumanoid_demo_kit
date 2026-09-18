#!/usr/bin/env bash
set -euo pipefail

SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET_HOME="${TARGET_HOME:-/home/unitree}"

SDK_SRC="$SRC_DIR/robot_home/unitree_sdk2_python_custom"
JOINTS_SRC="$SRC_DIR/robot_home/joints_xyz"
DFTP_SRC="$SRC_DIR/robot_home/DFTP_arm"
URDF_SRC="$SRC_DIR/robot_home/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf"

SDK_DST="$TARGET_HOME/unitree_sdk2_python_custom"
JOINTS_DST="$TARGET_HOME/joints_xyz"
DFTP_DST="$TARGET_HOME/DFTP_arm"
URDF_DST="$TARGET_HOME/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf"

echo "[install] Source repo: $SRC_DIR"
echo "[install] Target home: $TARGET_HOME"

for p in "$SDK_SRC" "$JOINTS_SRC" "$DFTP_SRC" "$URDF_SRC"; do
  if [ ! -e "$p" ]; then
    echo "[ERROR] Missing source: $p" >&2
    exit 1
  fi
done

mkdir -p "$SDK_DST" "$JOINTS_DST" "$DFTP_DST"

echo "[install] Copy unitree_sdk2_python_custom -> $SDK_DST"
rsync -avh --delete \
  --exclude='.git/***' \
  --exclude='build/***' \
  --exclude='install/***' \
  --exclude='log/***' \
  --exclude='.venv/***' \
  --exclude='**/__pycache__/***' \
  --exclude='**/*.pyc' \
  "$SDK_SRC/" "$SDK_DST/"

echo "[install] Copy joints_xyz -> $JOINTS_DST"
rsync -avh --delete \
  --exclude='**/__pycache__/***' \
  --exclude='**/*.pyc' \
  "$JOINTS_SRC/" "$JOINTS_DST/"

echo "[install] Copy DFTP_arm -> $DFTP_DST"
rsync -avh --delete \
  --exclude='.git/***' \
  --exclude='**/__pycache__/***' \
  --exclude='**/*.pyc' \
  --exclude='**/*.log' \
  "$DFTP_SRC/" "$DFTP_DST/"

echo "[install] Copy URDF -> $URDF_DST"
cp "$URDF_SRC" "$URDF_DST"

chmod +x "$SDK_DST/example/g1/high_level/"*.py 2>/dev/null || true
chmod +x "$JOINTS_DST/"*.py 2>/dev/null || true
chmod +x "$DFTP_DST/"*.py 2>/dev/null || true

cat > "$TARGET_HOME/g1_json_demo_env.sh" <<'EOF'
#!/usr/bin/env bash

# Common demo environment for Ubuntu 22 / ROS 2 Humble and Ubuntu 20 / ROS 2 Foxy.
# This file is optional for pure JSON playback from the high_level folder, but useful
# when ROS 2 recording or imports need a clean environment.

export PYTHONPATH="$HOME/unitree_sdk2_python_custom:${PYTHONPATH:-}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
  export G1_DEMO_ROS_DISTRO=humble
elif [ -f /opt/ros/foxy/setup.bash ]; then
  source /opt/ros/foxy/setup.bash
  export G1_DEMO_ROS_DISTRO=foxy
else
  echo "[WARN] No ROS2 setup found: /opt/ros/humble/setup.bash or /opt/ros/foxy/setup.bash" >&2
fi

if [ -f "$HOME/unitree_ros2/cyclonedds_ws/install/unitree_hg/share/unitree_hg/local_setup.bash" ]; then
  source "$HOME/unitree_ros2/cyclonedds_ws/install/unitree_hg/share/unitree_hg/local_setup.bash"
elif [ -f "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash" ]; then
  source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
fi
EOF
chmod +x "$TARGET_HOME/g1_json_demo_env.sh"

echo
echo "[check] Core files:"
for f in \
  "$SDK_DST/example/g1/high_level/joystick_launch_json_record.py" \
  "$SDK_DST/example/g1/high_level/record_actual_motion_ros.py" \
  "$SDK_DST/example/g1/high_level/g1_json_upper.py" \
  "$SDK_DST/example/g1/high_level/dataset_100/wave_right.json" \
  "$SDK_DST/example/g1/high_level/dataset_100/hold_box.json" \
  "$JOINTS_DST/lowstate_to_jointstates_ros.py" \
  "$URDF_DST" \
  "$DFTP_DST/sliders_arm.py" \
  "$DFTP_DST/hand_remote_poses.py"
do
  [ -e "$f" ] && echo "OK   $f" || echo "MISS $f"
done

echo
echo "[check] Unitree SDK import:"
PYTHONPATH="$SDK_DST:${PYTHONPATH:-}" python3 - <<'PY' || true
try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    print("OK unitree_sdk2py imports")
except Exception as e:
    print("MISS unitree_sdk2py import:", repr(e))
PY

echo
echo "[check] RH56DFTP/Register imports:"
python3 - <<'PY' || true
try:
    import RH56DFTP.RH56DFTP_TCP as tcp
    import Register.RegisterKey.ftp_registers_keys as regs
    print("OK RH56DFTP:", tcp.__file__)
    print("OK Register:", regs.__file__)
except Exception as e:
    print("MISS RH56DFTP/Register packages:", repr(e))
    print("Hand-control scripts need these packages installed separately.")
PY

echo
echo "[check] ROS2 setup auto-detect:"
bash -lc "source '$TARGET_HOME/g1_json_demo_env.sh'; echo ROS_DISTRO=\${G1_DEMO_ROS_DISTRO:-none}; echo RMW_IMPLEMENTATION=\${RMW_IMPLEMENTATION:-none}" || true

echo
echo "[done] Installed Unitree G1 JSON demo kit."
echo "[hint] Optional: source ~/g1_json_demo_env.sh"
