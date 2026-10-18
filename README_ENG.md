# WearBench-G1: Unitree G1 Telemetry Recorder

[Русская версия](README.md)

Tools for recording and validating Unitree G1 joint actuator and inertial sensor telemetry. The datasets can be used to analyze mechanical load indicators and compare real measurements with simulation.

## Compatibility

| Operating system | Python | ROS 2 |
| --- | --- | --- |
| Ubuntu 20.04 | 3.8 | Foxy |
| Ubuntu 22.04 | 3.10 | Humble |

The recorder uses Unitree SDK2 and DDS. ROS 2 is not required for receiving `LowState`.

An existing working `unitree_sdk2py` and CycloneDDS installation is required, together with network access to the robot. These scripts do not install dependencies or modify the system environment. The source targets Python 3.8 and 3.10; end-to-end DDS connectivity must be checked on the target robot.

## Files

| File | Purpose |
| --- | --- |
| `record_lowstate.py` | Record `LowState` messages to CSV and JSON metadata |
| `check_environment.py` | Check Python, SDK2, CycloneDDS and network interface |
| `validate_run.py` | Validate CSV integrity and timestamp characteristics |
| `README.md` | Russian documentation |

## Clone

```bash
cd ~
git clone --depth 1 --single-branch \
  -b wearbench-sensors-only \
  https://github.com/AlexArutiunian/neurohumanoid_demo_kit.git \
  wearbench_sensors
cd ~/wearbench_sensors
```

The tools locate SDK2 in the active Python environment or in `~/unitree_sdk2_python_custom` and `~/unitree_sdk2_python`. Use `--sdk-dir` for a different source directory.

## Environment check

```bash
python3 --version
python3 -m py_compile record_lowstate.py validate_run.py check_environment.py
python3 record_lowstate.py --self-test
ip -br addr
python3 check_environment.py --iface eth0
```

Set `--iface` to the network interface connected to the robot. For an SDK2 source tree in another directory:

```bash
python3 check_environment.py \
  --iface eth0 \
  --sdk-dir ~/unitree_sdk2_python_custom
```

## Recording

For the initial acquisition, keep the robot stationary:

```bash
python3 record_lowstate.py \
  --iface eth0 \
  --seconds 30 \
  --label stand_001 \
  --out-dir ~/wearbench_g1_data
```

Arguments:

- `--iface`: network interface
- `--topic`: DDS topic, default `rt/lowstate`
- `--seconds`: recording duration
- `--label`: experiment identifier
- `--out-dir`: output directory
- `--sdk-dir`: existing SDK2 directory

The recorder produces `*.csv` and `*.meta.json`. The CSV contains timestamps, robot tick, IMU readings and the following fields for 29 actuators:

`q`, `dq`, `ddq`, `tau_est`, `temperature_0`, `temperature_1`, `vol`, `mode`, `motorstate`.

The metadata records acquisition settings, message counts, queue drops and decoding errors.

## Validation

```bash
ls -lh ~/wearbench_g1_data

python3 validate_run.py \
  ~/wearbench_g1_data/RECORDING_NAME.csv
```

The validator checks the CSV schema, timestamps, sample sequence and principal numerical fields. The report includes the observed sampling rate.

If no data arrives, verify the network interface, active topic and DDS configuration. Use `rt/lf/lowstate` only after confirming the publisher and message type.

## Measurement limitations

- `tau_est` is an SDK-provided torque estimate, not calibrated shaft-torque measurement
- Temperature fields are stored as raw SDK values
- The `LowState_` message does not provide a separate per-actuator current measurement
- Healthy-robot load recordings do not identify physical actuator lifetime

## Safety

The recorder only subscribes through `ChannelSubscriber`. It does not publish control commands, change robot modes or initiate motion.

Initial testing should be performed with the robot stationary. Subsequent recordings during motion must follow the robot's normal operating and safety procedures.
