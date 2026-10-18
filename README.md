# Neurohumanoid Demo Kit - управление Unitree G1 через JSON

[English version](README_ENG.md)

Набор файлов для переноса работающей системы управления верхней частью Unitree G1 через JSON на другого робота.

В репозитории собраны подготовленные файлы, которые используются на стороне робота:

- модифицированная версия Unitree SDK2 Python для демонстрации
- запуск движений из JSON по кнопкам джойстика
- запись фактических движений через ROS 2, топики `/joint_states` и `/tf`
- URDF-модель Unitree G1 с кистями Inspire
- примеры работы с RH56DFTP / Inspire и заметки по тактильным датчикам

## Дополнительно: WearBench-G1

В ветке `wearbench-g1-readonly-telemetry` есть отдельный регистратор для научного исследования нагрузок на суставы. Он читает `rt/lowstate`, записывает состояния 29 приводов и IMU и **не отправляет команд управления**.

- [Инструкция по записи телеметрии](robot_home/wearbench_g1/README.md)
- [Скрипт записи](robot_home/wearbench_g1/record_lowstate.py)
- [Проверка полученных данных](robot_home/wearbench_g1/validate_run.py)
- [История конфликтов управляющих процессов](debug_history/arm_sdk_control_conflict.md)

Для записи WearBench не нужно запускать установщик JSON-демонстрации или копировать файлы поверх рабочего окружения робота. Подробные команды приведены в отдельной инструкции.

## Совместимость Ubuntu и ROS 2

Воспроизведение JSON-движений руками в основном не зависит от Ubuntu 20 или Ubuntu 22. Главное, чтобы работала `unitree_sdk2py` и был указан правильный сетевой интерфейс.

Запись через ROS 2 зависит от установленной версии ROS:

- Ubuntu 22 обычно использует ROS 2 Humble: `/opt/ros/humble/setup.bash`
- Ubuntu 20 обычно использует ROS 2 Foxy: `/opt/ros/foxy/setup.bash`

В этой ветке Humble или Foxy определяются автоматически:

- в `robot_home/unitree_sdk2_python_custom/example/g1/high_level/joystick_launch_json_record.py` при запуске `record_actual_motion_ros.py`
- в файле `~/g1_json_demo_env.sh`, который создаётся скриптом `scripts/install_to_robot.sh`

Если Humble и Foxy не установлены, воспроизведение JSON всё равно можно запускать с параметром `--no-record`. Для записи через ROS 2 сначала потребуется установить и настроить ROS.

## Структура репозитория

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

## Что намеренно не включено

Тяжёлые и автоматически создаваемые файлы в набор не входят:

- виртуальные окружения Python: `.venv/`, `venv/`
- результаты сборки ROS/colcon: `build/`, `install/`, `log/`
- журналы движений и записанные наборы данных: `actual_motion_logs/`, `data_steer/`, `data_real_states/`, `data_real_keyframes/`
- RGB-D наборы данных: `dataset_rgbd/`
- архивы, временные и служебные файлы: `*.zip`, `*.bag`, `*.db3`, `*.log`, `*.pyc`

Цель набора - сохранить код и небольшие JSON-движения, а не старые записи или локальные окружения.

## Внешние зависимости кистей Inspire/RH56DFTP

В папке `DFTP_arm` есть примеры и инструменты для тактильных датчиков. При этом Python-пакеты, которые могут быть установлены на самом роботе, необязательно находятся внутри репозитория:

- `RH56DFTP`
- `Register`

Проверка на роботе:

```bash
python3 - <<'PY'
import RH56DFTP.RH56DFTP_TCP as tcp
import Register.RegisterKey.ftp_registers_keys as regs
print("OK RH56DFTP:", tcp.__file__)
print("OK Register:", regs.__file__)
PY
```

Если импорт не работает, отдельно установите или скопируйте Python API для Inspire RH56DFTP, прежде чем запускать управление кистями.

## Установка на другого робота

Для переноса именно **JSON-демонстрации** на новый Unitree G1:

```bash
cd ~
git clone -b json-ctrl https://github.com/AlexArutiunian/neurohumanoid_demo_kit.git
cd neurohumanoid_demo_kit
bash scripts/install_to_robot.sh
```

По умолчанию установщик копирует файлы в `/home/unitree`. Для другого домашнего каталога используйте:

```bash
TARGET_HOME=/home/unitree bash scripts/install_to_robot.sh
```

Установщик также создаёт:

```bash
~/g1_json_demo_env.sh
```

Для простого воспроизведения JSON из папки `high_level` этот файл необязателен, но он помогает настроить ROS и пути импорта:

```bash
source ~/g1_json_demo_env.sh
```

## Запуск JSON-демонстрации с джойстика

### Терминал 1 - публикация `/joint_states`

```bash
source ~/g1_json_demo_env.sh
cd ~/joints_xyz
python3 lowstate_to_jointstates_ros.py
```

### Терминал 2 - публикация состояния робота

```bash
source ~/g1_json_demo_env.sh
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p robot_description:="$(cat ~/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf)"
```

### Терминал 3 - запуск скрипта джойстика

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

Если запись через ROS не нужна, можно запустить тот же скрипт без неё:

```bash
cd ~/unitree_sdk2_python_custom/example/g1/high_level

python3 joystick_launch_json_record.py eth0 \
  --dataset-dir dataset_100 \
  --json-a wave_right.json \
  --json-b hold_box.json \
  --json-down 0.json \
  --no-record
```

Назначение кнопок в `joystick_launch_json_record.py`:

- `A` - запустить JSON, выбранный параметром `--json-a`
- `B` - запустить JSON, выбранный параметром `--json-b`
- `Down` - запустить JSON, выбранный параметром `--json-down`
- `Up` - пропустить текущий пустой кадр удержания в JSON-плеере
- `X` - разжать пальцы правой кисти
- `Y` - сжать правую кисть в кулак
- `Select + Start` - выйти из программы

## Быстрые проверки после установки

Проверьте наличие основных файлов:

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

Проверьте выбранное окружение ROS 2:

```bash
source ~/g1_json_demo_env.sh
echo "ROS_DISTRO=${G1_DEMO_ROS_DISTRO:-none}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-none}"
```

Проверьте импорт Unitree SDK:

```bash
cd ~/unitree_sdk2_python_custom
python3 - <<'PY'
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
print("OK unitree_sdk2py imports")
PY
```

Проверьте импорт пакетов для кисти RH56DFTP:

```bash
python3 - <<'PY'
import RH56DFTP.RH56DFTP_TCP as tcp
import Register.RegisterKey.ftp_registers_keys as regs
print("OK RH56DFTP:", tcp.__file__)
print("OK Register:", regs.__file__)
PY
```

## Безопасность

Не запускайте старые скрипты прямого низкоуровневого управления руками параллельно с `joystick_launch_json_record.py` или `g1_json_upper.py`.

Не запускайте `sliders_arm.py` одновременно с программой джойстика, если обе программы пытаются управлять одной и той же кистью RH56DFTP по TCP.

На новом роботе начинайте с коротких и простых JSON-движений. Во время проверки должен быть доступ к аварийной остановке.

Подробнее об уже возникавших конфликтах процессов: [debug_history/arm_sdk_control_conflict.md](debug_history/arm_sdk_control_conflict.md).
