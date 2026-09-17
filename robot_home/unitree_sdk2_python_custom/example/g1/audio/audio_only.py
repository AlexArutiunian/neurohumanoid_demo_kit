#!/usr/bin/env python3

import sys
import time
import subprocess
from pathlib import Path


def main():
    network_interface = sys.argv[1] if len(sys.argv) > 1 else "eth0"

    current_dir = Path(__file__).resolve().parent
    player_script = current_dir / "g1_audio_client_play_wav.py"

    audio_files = [
        current_dir / "1.wav",
        current_dir / "2.wav",
    ]

    if not player_script.is_file():
        print(f"[ERROR] Не найден проигрыватель: {player_script}")
        sys.exit(1)

    for audio_file in audio_files:
        if not audio_file.is_file():
            print(f"[ERROR] Не найден аудиофайл: {audio_file}")
            sys.exit(1)

    print("[INFO] Запуск первой реплики...")

    subprocess.run(
        [
            sys.executable,
            str(player_script),
            network_interface,
            str(audio_files[0]),
        ],
        check=True,
    )

    # Пауза между двумя репликами.
    time.sleep(1.0)

    print("[INFO] Запуск второй реплики...")

    subprocess.run(
        [
            sys.executable,
            str(player_script),
            network_interface,
            str(audio_files[1]),
        ],
        check=True,
    )

    print("[SUCCESS] Обе реплики воспроизведены.")


if __name__ == "__main__":
    main()
