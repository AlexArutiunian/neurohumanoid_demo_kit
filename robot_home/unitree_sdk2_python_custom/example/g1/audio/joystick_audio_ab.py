#!/usr/bin/env python3

import sys
import time
import queue
import subprocess
import threading
from pathlib import Path

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_


# Файлы, которые ты скопировал в /home/unitree
AUDIO_A = Path("/home/unitree/1_g1_loud.wav")
AUDIO_B = Path("/home/unitree/2_g1_loud.wav")

# Уже проверенный проигрыватель WAV
PLAYER = Path(__file__).resolve().parent / "g1_audio_client_play_wav.py"

jobs = queue.Queue(maxsize=10)

previous_a = 0
previous_b = 0
button_lock = threading.Lock()


def lowstate_handler(msg: LowState_):
    """Обрабатывает нажатия A и B штатного пульта Unitree."""

    global previous_a, previous_b

    remote = msg.wireless_remote

    if remote is None or len(remote) < 4:
        return

    # remote[3]:
    # bit 0 = A
    # bit 1 = B
    current_a = (int(remote[3]) >> 0) & 1
    current_b = (int(remote[3]) >> 1) & 1

    with button_lock:
        # Срабатывание только в момент нажатия, а не пока кнопка удерживается
        if current_a == 1 and previous_a == 0:
            print("\n[BUTTON] A -> запуск 1_g1_loud.wav")
            try:
                jobs.put_nowait(("A", AUDIO_A))
            except queue.Full:
                print("[WARNING] Очередь воспроизведения заполнена")

        if current_b == 1 and previous_b == 0:
            print("\n[BUTTON] B -> запуск 2_g1_loud.wav")
            try:
                jobs.put_nowait(("B", AUDIO_B))
            except queue.Full:
                print("[WARNING] Очередь воспроизведения заполнена")

        previous_a = current_a
        previous_b = current_b


def audio_worker(network_interface: str):
    """Последовательно воспроизводит поставленные в очередь файлы."""

    while True:
        button_name, audio_path = jobs.get()

        try:
            if not audio_path.is_file():
                print(f"[ERROR] Файл для кнопки {button_name} не найден:")
                print(f"        {audio_path}")
                continue

            print(f"[AUDIO] Воспроизведение: {audio_path}")

            result = subprocess.run(
                [
                    sys.executable,
                    str(PLAYER),
                    network_interface,
                    str(audio_path),
                ],
                check=False,
            )

            if result.returncode == 0:
                print(f"[OK] Реплика кнопки {button_name} завершена")
            else:
                print(
                    f"[ERROR] Проигрыватель завершился с кодом "
                    f"{result.returncode}"
                )

        except Exception as error:
            print(f"[ERROR] Не удалось проиграть файл: {error}")

        finally:
            jobs.task_done()


def main():
    network_interface = sys.argv[1] if len(sys.argv) > 1 else "eth0"

    if not PLAYER.is_file():
        print(f"[ERROR] Не найден проигрыватель:")
        print(f"        {PLAYER}")
        sys.exit(1)

    missing_files = [
        path for path in (AUDIO_A, AUDIO_B)
        if not path.is_file()
    ]

    if missing_files:
        print("[ERROR] Не найдены аудиофайлы:")
        for path in missing_files:
            print(f"        {path}")
        sys.exit(1)

    print(f"[INFO] Сетевой интерфейс: {network_interface}")
    print(f"[INFO] A -> {AUDIO_A}")
    print(f"[INFO] B -> {AUDIO_B}")
    print("[INFO] Управления движениями в этом скрипте нет")

    ChannelFactoryInitialize(0, network_interface)

    worker = threading.Thread(
        target=audio_worker,
        args=(network_interface,),
        daemon=True,
    )
    worker.start()

    subscribers = []

    # Основной топик для G1
    for topic in ("rt/lf/lowstate", "rt/lowstate"):
        try:
            subscriber = ChannelSubscriber(topic, LowState_)
            subscriber.Init(lowstate_handler, 10)
            subscribers.append(subscriber)
            print(f"[INFO] Подписка на {topic}")
        except Exception as error:
            print(f"[WARNING] Не удалось подписаться на {topic}: {error}")

    if not subscribers:
        print("[ERROR] Не удалось создать подписку LowState")
        sys.exit(1)

    print()
    print("==========================================")
    print("  A — первая речь")
    print("  B — вторая речь")
    print("  Ctrl+C — выход")
    print("==========================================")

    try:
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n[INFO] Скрипт остановлен")


if __name__ == "__main__":
    main()
