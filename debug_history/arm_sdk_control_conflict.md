# Debug history: `rt/arm_sdk` control conflict / joint buzzing

This note documents a past dangerous bug so it is not reintroduced when moving the JSON-control demo to another Unitree G1 robot.

## Symptom

The robot could start hissing/buzzing with the upper-body joints and behave as if the arms were being pulled by competing controllers.

Typical visible symptoms:

- arm/waist joints buzz or vibrate instead of smoothly holding a pose;
- the robot resists motion or twitches around a commanded pose;
- starting another JSON motion while an old script is still alive makes the issue worse;
- killing one of the control processes stops or reduces the noise.

## Root cause

The problem was not a broken motor. The dangerous case was multiple active processes publishing control commands for the same upper-body joints at the same time.

The main hazardous pattern was:

1. a joystick/button script also created its own arm-control loop;
2. that script repeatedly published `LowCmd_` commands to the Unitree arm SDK topic, usually `rt/arm_sdk`;
3. another process, such as a JSON player, was launched and also started commanding the same joints;
4. the two controllers fought each other.

That kind of conflict can produce buzzing, hissing, twitching, high actuator load, and unsafe motion.

## Related joystick D-pad hazard: arrow sequences and accidental commands

A second historical issue involved using the wireless-controller D-pad arrows as motion-launch controls.

Observed dangerous case:

- a `Left -> Right` arrow sequence was used around a launch path for `35.json`;
- after that sequence, the robot behavior became harsh/uncontrolled;
- direct manual runs of `g1_json_upper.py` could finish normally, but launching through the joystick path reproduced bad behavior;
- the suspected mechanism was that D-pad arrow presses were also reaching the robot's stock/low-level controller as movement/turn/body commands while the custom arm demo was being started.

Do **not** treat D-pad arrows as harmless generic buttons. On the G1 wireless controller they may have meaning outside this Python launcher, depending on robot mode and what other Unitree services are active.

Bad pattern:

```text
Left arrow  -> prepare/retake/previous/mode switch
Right arrow -> confirm/launch/next/mode switch
Left then Right quickly -> launches a motion while also injecting arrow commands into the robot controller path
```

Risk:

```text
custom launcher state machine changes + stock controller D-pad handling + arm_sdk JSON control
```

This can create confusing state transitions and motion-controller overlap. It is especially bad when the launch action happens immediately on press instead of after a clean release/debounce interval.

## Joystick mapping rules

Preferred mapping for demo motions:

- use `A`, `B`, and optionally `Down` for explicit JSON selection;
- use `X`/`Y` only for RH56DFTP hand TCP poses;
- use `Up` only for the narrow non-motion action: skip the current empty JSON hold frame by sending newline to the already-running player;
- avoid `Left` and `Right` for starting arm motions;
- avoid multi-arrow sequences such as `Left -> Right`, `Right -> Left`, `Up -> Down`, or `Down -> Up` for any action that can launch a new controller.

Safer launcher behavior:

1. detect a button edge;
2. store a pending launch label;
3. wait until the button is released;
4. wait a short debounce/settle interval, for example `--launch-after-release-sec 0.35`;
5. launch only if no other JSON player is active.

Do not launch a new arm-control process directly on a D-pad press.

Future rule: `Left` and `Right` should stay unassigned unless there is a strong reason and the behavior is tested on the real robot in the exact same mode. If they are ever used, they must not publish to `rt/arm_sdk` and must not launch another arm controller.

## Unsafe design pattern — do not reintroduce

Do **not** write a launcher that both:

- listens to joystick/buttons; and
- owns a recurrent arm/waist control loop that publishes to `rt/arm_sdk`; and
- launches another script that also controls the same joints.

Bad pattern in pseudocode:

```python
# DANGEROUS DESIGN
# One script keeps publishing joint commands...
arm_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
control_thread = RecurrentThread(interval=0.002, target=publish_lowcmd_forever)
control_thread.Start()

# ...and then launches another controller/player for the same joints.
subprocess.Popen(["python3", "g1_json_upper.py", "eth0", "--dataset-path", "..."])
```

This creates two independent sources of commands for the same joints.

## Safer current architecture

The safer architecture used in this branch is:

```text
joystick_launch_json_record.py
    listens to joystick
    starts/stops recorder
    launches exactly one g1_json_upper.py process at a time
    does not continuously publish arm joint LowCmd commands itself

        |
        v

g1_json_upper.py
    owns the arm/waist control loop while the JSON is running
    publishes the upper-body commands
    returns to start pose and releases arm_sdk weight at the end
```

The launcher should be an orchestrator, not a second arm controller.

## Rules for future changes

1. Only one active upper-body joint controller should publish commands for the same joints at a time.
2. A joystick launcher may listen to `LowState_` / `wireless_remote`, but it should not continuously command arm joints if it is also launching `g1_json_upper.py`.
3. If a new controller is added, define ownership clearly: which process owns `rt/arm_sdk`, and when.
4. Avoid running these together unless you are certain they do not command the same joints:
   - `g1_json_upper.py`
   - `g1_arm*_example.py`
   - `hold_waist.py`
   - old joystick/button arm-control scripts
   - any script with `ChannelPublisher("rt/arm_sdk", LowCmd_)`
   - any script using `RecurrentThread` to publish joint commands
5. `sliders_arm.py` is for the RH56DFTP hand TCP control, not Unitree arm joints, but do not run multiple hand TCP clients at the same time if they control the same hand.
6. Do not map `Left`/`Right` D-pad sequences to motion launch or controller-mode changes without a real-robot safety review.

## Pre-flight check before demos

Run this before launching the demo:

```bash
ps aux | grep -Ei 'g1_json|joystick_launch|g1_arm|hold_waist|sliders_arm' | grep -v grep
```

If stale controllers are running, stop them:

```bash
pkill -f 'g1_json'
pkill -f 'joystick_launch'
pkill -f 'g1_arm'
pkill -f 'hold_waist'
# Only if the hand slider should not be active:
pkill -f 'sliders_arm'
```

Then start only the intended launcher:

```bash
cd ~/unitree_sdk2_python_custom/example/g1/high_level

python3 joystick_launch_json_record.py eth0 \
  --dataset-dir dataset_100 \
  --json-a wave_right.json \
  --json-b hold_box.json \
  --json-down 0.json \
  --kp 60 \
  --kd 1.5 \
  --kp-waist 250 \
  --kd-waist 6 \
  --record-out-dir ./actual_motion_logs \
  --record-rate-hz 50 \
  --record-target-frame pelvis
```

## Code review checklist

Before merging any future joystick/control changes, check:

```bash
grep -R "ChannelPublisher\|rt/arm_sdk\|RecurrentThread\|LowCmd_" -n \
  robot_home/unitree_sdk2_python_custom/example/g1/high_level \
  | grep -v g1_json_upper.py
```

If this finds a launcher or helper that publishes upper-body joint commands, review it very carefully. It may reintroduce the old conflict.

Also check joystick-arrow usage:

```bash
grep -R "Left\|Right\|Up\|Down\|launch-after-release\|pending\|release_since" -n \
  robot_home/unitree_sdk2_python_custom/example/g1/high_level/joystick_launch*.py
```

If `Left` or `Right` can launch motion or change controller state, treat it as a safety-critical change.

## Emergency recovery during testing

If the robot starts buzzing or behaving wrong:

1. Stop the active script with `Ctrl+C`.
2. Kill stale processes:

```bash
pkill -f 'g1_json'
pkill -f 'joystick_launch'
pkill -f 'g1_arm'
pkill -f 'hold_waist'
```

3. Wait until the robot is stable.
4. Restart with only one controller active.

Do not repeatedly relaunch JSON motions while the cause is unknown.

## Historical note

This repository keeps the debug note because the failure was easy to reproduce accidentally during fast demo iteration: joystick state-machine edits, D-pad mappings, and arm-control ownership were changed together. Future work should keep those concerns separate:

```text
joystick reader / state machine  !=  arm_sdk owner
D-pad navigation                 !=  motion launch
hand TCP pose commands           !=  Unitree upper-body joint control
```
