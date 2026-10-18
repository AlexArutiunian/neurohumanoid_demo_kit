# WearBench-G1 - только датчики

[English README](README_ENG.md)

Минимальная ветка для пассивной записи телеметрии исправного Unitree G1. **Никаких контроллеров, JSON-плееров, Inspire, URDF, ROS-проектов или установщиков здесь нет.**

Работает с уже установленным Unitree SDK2 на Ubuntu 20.04 (Python 3.8, ROS Foxy) и Ubuntu 22.04 (Python 3.10, ROS Humble). Сам регистратор использует DDS напрямую, **ROS ему не нужен**.

## Состав

```text
README.md
README_ENG.md
record_lowstate.py
validate_run.py
check_environment.py
.gitignore
```

- `record_lowstate.py` - слушает `rt/lowstate`, записывает CSV и JSON с метаданными
- `check_environment.py` - проверяет SDK, CycloneDDS, Python и сетевой интерфейс, ничего не устанавливает
- `validate_run.py` - проверяет готовый CSV без робота и SDK

## Установка

Клонируйте **именно эту ветку**, без огромной истории:

```bash
cd ~
git clone --depth 1 --single-branch -b wearbench-sensors-only \
  https://github.com/AlexArutiunian/neurohumanoid_demo_kit.git wearbench_sensors
cd ~/wearbench_sensors
```

Ничего устанавливать поверх работающей системы робота **не нужно**. Скрипт сначала ищет уже имеющийся SDK в `~/unitree_sdk2_python_custom` или `~/unitree_sdk2_python`, а затем использует установленный пакет из текущего Python.

Если SDK находится в другом месте, используйте `--sdk-dir /путь/к/unitree_sdk2_python_custom`.

## Быстрая проверка без робота

```bash
python3 --version
python3 -m py_compile record_lowstate.py validate_run.py check_environment.py
python3 record_lowstate.py --self-test
ip -br addr
python3 check_environment.py --iface eth0
```

Если `eth0` не существует, подставьте сетевой интерфейс робота.

## Первая запись: только неподвижный робот

```bash
python3 record_lowstate.py \
  --iface eth0 \
  --seconds 30 \
  --label stand_001 \
  --out-dir ~/wearbench_g1_data
```

После записи проверьте имя файла и качество:

```bash
ls -lh ~/wearbench_g1_data
python3 validate_run.py ~/wearbench_g1_data/ИМЯ_ФАЙЛА.csv
```

В CSV есть временные метки, `tick`, `q`, `dq`, `ddq`, `tau_est`, два температурных поля, `vol`, `mode`, `motorstate` для 29 суставов и данные IMU. Рядом сохраняется `.meta.json` со счётчиками пропущенных сообщений и ошибками.

**Важно:** `tau_est` - оценка SDK, не калиброванный датчик момента. Поля температуры сохраняются без предположений о физическом смысле. Токи моторов данным типом `LowState_` не передаются. Эти записи позволяют исследовать нагрузки на здоровом G1, но не определять фактический срок службы привода.

## Если что-то не работает

Если Python не импортирует `unitree_sdk2py`, укажите местоположение существующего SDK:

```bash
python3 check_environment.py --iface eth0 \
  --sdk-dir ~/unitree_sdk2_python_custom

python3 record_lowstate.py --iface eth0 \
  --sdk-dir ~/unitree_sdk2_python_custom \
  --seconds 10 --label check
```

Если после 10 секунд нет сообщений, проверьте интерфейс и активный DDS-топик на вашем G1. Другой вариант `--topic rt/lf/lowstate` используйте только после проверки типа и источника сообщений. Не меняйте ROS или CycloneDDS в рабочем окружении без диагностики.

## Безопасность

Программа использует **только `ChannelSubscriber`**. Она не содержит `ChannelPublisher`, не публикует `LowCmd`, не обращается к `rt/arm_sdk`, не запускает движения и не переключает режимы G1. Можно логировать штатные движения, уже запущенные вашим единственным контроллером, не начиная второй контроллер.

На физическом G1 реализация пока не проверена. Первый тест проводите на неподвижном роботе. Данные должны пройти проверку перед использованием в статье.
