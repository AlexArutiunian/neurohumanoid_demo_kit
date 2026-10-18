# WearBench-G1 - Sensors Only

Minimal read-only Unitree G1 telemetry recorder for an existing robot environment. No motion control, joystick software, hands, URDF files, ROS projects or installers.

**Supported targets:** Ubuntu 20.04 with Python 3.8 / ROS Foxy and Ubuntu 22.04 with Python 3.10 / ROS Humble. The recorder uses Unitree SDK2 DDS directly; ROS 2 is not required.

## Contents

- `record_lowstate.py` - capture 29 actuators plus IMU from `rt/lowstate` into CSV and JSON metadata
- `check_environment.py` - inspect Python, CycloneDDS, the installed SDK and network interface
- `validate_run.py` - offline CSV validation with Python standard library only
- `README.md` - detailed Russian guide

## Clone only this branch

```bash
cd ~
git clone --depth 1 --single-branch -b wearbench-sensors-only \
  https://github.com/AlexArutiunian/neurohumanoid_demo_kit.git wearbench_sensors
cd ~/wearbench_sensors
```

No installer and no SDK/ROS changes are needed. The scripts try `~/unitree_sdk2_python_custom`, then `~/unitree_sdk2_python`, then the active Python environment. Use `--sdk-dir /existing/sdk/directory` if needed.

## Test and collect

```bash
python3 -m py_compile record_lowstate.py validate_run.py check_environment.py
python3 record_lowstate.py --self-test
python3 check_environment.py --iface eth0
python3 record_lowstate.py --iface eth0 --seconds 30 --label stand_001 \
  --out-dir ~/wearbench_g1_data
ls ~/wearbench_g1_data
python3 validate_run.py ~/wearbench_g1_data/RECORDING_NAME.csv
```

Change `eth0` to the actual interface. For import errors, use `--sdk-dir` pointing to your already installed Unitree SDK2 source tree. If no DDS messages are received, verify the interface and topic. Do not replace the production Python/CycloneDDS environment.

**Safety:** this branch never issues robot commands. It subscribes to LowState only. The motor `tau_est` and temperature fields are recorded raw and must not be interpreted as calibrated mechanical wear or physical lifetime estimates. Real G1 connectivity has not yet been hardware-tested with this script.
