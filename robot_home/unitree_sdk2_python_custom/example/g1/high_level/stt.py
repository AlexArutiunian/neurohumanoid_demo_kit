from __future__ import annotations

import argparse
import json
import os
import queue
from pathlib import Path

import sounddevice as sd
from vosk import KaldiRecognizer, Model

DEFAULT_SAMPLE_RATE = 16000
DEFAULT_BLOCK_SIZE = 8000
GRAMMAR = ["левая рука вверх", "правая рука вверх", "обе руки вверх", "привет", "[unk]"]

audio_queue = queue.Queue()


def resolve_model_path(cli_path: str | None) -> str:
    if cli_path:
        return cli_path

    env_path = os.getenv("VOSK_MODEL_PATH")
    if env_path:
        return env_path

    base = Path(__file__).resolve().parent
    candidates = [base / "vosk-ru", base / "vosk-model-ru-0.22", base / "vosk-model-small-ru-0.22"]
    for candidate in candidates:
        if candidate.exists():
            return str(candidate)
    return str(base / "vosk-ru")


def print_input_devices() -> None:
    devices = sd.query_devices()
    for idx, dev in enumerate(devices):
        if dev["max_input_channels"] > 0:
            print(f"[{idx}] {dev['name']} (inputs={dev['max_input_channels']}, default_sr={dev['default_samplerate']})")


def audio_callback(indata, frames, time_info, status):
    if status:
        print(status)
    audio_queue.put(bytes(indata))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Offline STT (Vosk) from microphone")
    parser.add_argument("--model-path", default="/home/unitree/unitree_sdk2_python/example/g1/high_level/vosk-ru", help="Path to Vosk model directory")
    parser.add_argument("--samplerate", type=int, default=DEFAULT_SAMPLE_RATE, help="Input sample rate (Hz)")
    parser.add_argument("--blocksize", type=int, default=DEFAULT_BLOCK_SIZE, help="PortAudio block size")
    parser.add_argument("--device", default=None, help="Input device index or name (see --list-devices)")
    parser.add_argument("--list-devices", action="store_true", help="Print available input devices and exit")
    return parser


def main():
    args = build_parser().parse_args()

    if args.list_devices:
        print_input_devices()
        return

    model_path = resolve_model_path(args.model_path)
    if not Path(model_path).exists():
        raise FileNotFoundError(
            f"Vosk model not found: {model_path}. "
            f"Set --model-path or VOSK_MODEL_PATH."
        )

    model = Model(model_path)
    recognizer = KaldiRecognizer(model, args.samplerate, json.dumps(GRAMMAR, ensure_ascii=False))

    if args.device is not None:
        try:
            device = int(args.device)
        except ValueError:
            device = args.device
    else:
        device = None

    if device is not None:
        dev_info = sd.query_devices(device, "input")
        print(f"Input device: {dev_info['name']} (sr={args.samplerate})")

    last_partial = ""
    try:
        with sd.RawInputStream(
            samplerate=args.samplerate,
            blocksize=args.blocksize,
            dtype="int16",
            channels=1,
            callback=audio_callback,
            device=device,
        ):
            print("Слушаю... Ctrl+C для выхода")
            while True:
                data = audio_queue.get()
                if recognizer.AcceptWaveform(data):
                    result = json.loads(recognizer.Result())
                    text = result.get("text", "").strip()
                    if text:
                        print(" " * 100, end="\r")
                        print("FINAL:", text)
                    last_partial = ""
                else:
                    partial = json.loads(recognizer.PartialResult()).get("partial", "").strip()
                    if partial and partial != last_partial:
                        print(f"PARTIAL: {partial}", end="\r", flush=True)
                        last_partial = partial
    except KeyboardInterrupt:
        print("\nОстановлено.")
    except Exception as exc:
        print(f"Ошибка аудиоввода: {exc}")
        print("Подсказка: запустите с --list-devices и выберите --device <index>.")


if __name__ == "__main__":
    main()

