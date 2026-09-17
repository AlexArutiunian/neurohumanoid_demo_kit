#!/usr/bin/env python3
# Voice control for Inspire Hand:
# Shift+Space -> start dictation
# Space -> stop dictation and execute command via LLM

import argparse
import io
import json
import logging
import os
import queue
import re
import sys
import tempfile
import threading
import time
import termios
import tty

import numpy as np
from RH56DFTP.RH56DFTP_TCP import RH56DFTP_TCP
from Register.RegisterKey.ftp_registers_keys import (
    POS_SET_0,
    POS_SET_1,
    POS_SET_2,
    POS_SET_3,
    POS_SET_4,
    POS_SET_5,
)

try:
    from DFTP_arm.vsellm import get_chat_model, get_stt_model, get_vsellm_client
except Exception:
    from vsellm import get_chat_model, get_stt_model, get_vsellm_client

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8")
sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8")

MOVEMENT_DELAY = 0.2
FINGER_REGS = [POS_SET_0, POS_SET_1, POS_SET_2, POS_SET_3, POS_SET_4, POS_SET_5]

POSES = {
    "1": {"name": "Открытая ладонь", "positions": [0, 0, 0, 0, 0, 0]},
    "2": {"name": "Кулак", "positions": [1800, 1800, 1800, 1800, 1800, 0]},
    "3": {"name": "Победа/мир", "positions": [1800, 1800, 0, 0, 1800, 0]},
    "4": {"name": "Рок", "positions": [0, 1800, 1800, 0, 1800, 0]},
    "5": {"name": "Большой палец вверх", "positions": [1800, 1800, 1800, 1800, 0, 0]},
    "6": {"name": "Указание", "positions": [1800, 1800, 1800, 0, 1800, 0]},
    "7": {"name": "OK", "positions": [0, 0, 0, 1200, 500, 2000]},
}


def beep() -> None:
    print("\a", end="", flush=True)


def set_positions(client: RH56DFTP_TCP, positions):
    for reg, pos in zip(FINGER_REGS, positions):
        client.set(reg, int(pos))
        time.sleep(MOVEMENT_DELAY)


def execute_pose(client: RH56DFTP_TCP, pose_key: str) -> bool:
    pose = POSES.get(pose_key)
    if not pose:
        print(f"❌ Неизвестная поза: {pose_key}")
        return False

    print(f"🎭 Выполняю: {pose['name']} (#{pose_key})")

    # More stable order for OK gesture.
    if pose_key == "7":
        client.set("SPEED_SET(3)", 220)
        client.set("SPEED_SET(4)", 220)
        client.set("SPEED_SET(5)", 180)
        client.set("FORCE_SET(3)", 400)
        client.set("FORCE_SET(4)", 550)
        client.set("FORCE_SET(5)", 350)
        # Open non-participating fingers first.
        set_positions(client, [0, 0, 0, pose["positions"][3], pose["positions"][4], pose["positions"][5]])
    else:
        set_positions(client, pose["positions"])

    print("✅ Готово")
    return True


def reset_pose(client: RH56DFTP_TCP) -> None:
    execute_pose(client, "1")


class HotkeyState:
    def __init__(self):
        self.shift_pressed = False
        self.recording = False
        self.start_event = threading.Event()
        self.stop_event = threading.Event()
        self.mode = "unknown"
        self.command_queue = queue.Queue()


def make_keyboard_listener(state: HotkeyState):
    try:
        from pynput import keyboard  # type: ignore

        shift_keys = {keyboard.Key.shift, keyboard.Key.shift_l, keyboard.Key.shift_r}

        def on_press(key):
            if key in shift_keys:
                state.shift_pressed = True
                return

            if key == keyboard.Key.space:
                if state.shift_pressed and not state.recording:
                    state.start_event.set()
                elif state.recording:
                    state.stop_event.set()
                return

            # Direct command keys in voice mode.
            ch = getattr(key, "char", None)
            if not ch:
                return
            ch = ch.lower()
            if ch in "1234567":
                state.command_queue.put(f"pose {ch}")
            elif ch == "r":
                state.command_queue.put("open")
            elif ch == "g":
                state.command_queue.put("grasp")
            elif ch == "x":
                state.command_queue.put("exit")

        def on_release(key):
            if key in shift_keys:
                state.shift_pressed = False

        state.mode = "pynput"
        return keyboard.Listener(on_press=on_press, on_release=on_release)

    except Exception:
        # Fallback for headless/no-X sessions where pynput cannot attach.
        state.mode = "stdin"
        stop_flag = threading.Event()

        def _stdin_loop():
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                while not stop_flag.is_set():
                    ch = sys.stdin.read(1)
                    if not ch:
                        continue
                    # In raw TTY we usually cannot distinguish Shift+Space;
                    # use 's' to start recording in fallback mode.
                    if ch.lower() == "s" and not state.recording:
                        state.start_event.set()
                    elif ch == " " and state.recording:
                        state.stop_event.set()
                    elif ch in "1234567":
                        state.command_queue.put(f"pose {ch}")
                    elif ch.lower() == "r":
                        state.command_queue.put("open")
                    elif ch.lower() == "g":
                        state.command_queue.put("grasp")
                    elif ch.lower() == "x":
                        state.command_queue.put("exit")
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)

        t = threading.Thread(target=_stdin_loop, daemon=True)
        t.start()

        class _DummyListener:
            def start(self):
                return self

            def stop(self):
                stop_flag.set()

        return _DummyListener()


def record_audio_until_space(state: HotkeyState, samplerate: int, channels: int = 1):
    import sounddevice as sd

    frames = []

    def callback(indata, frames_count, time_info, status):
        del frames_count, time_info
        if status:
            print(f"[audio] {status}")
        frames.append(indata.copy())

    print("🎙️  Запись... Нажмите Space для завершения")
    beep()
    state.recording = True
    state.stop_event.clear()

    with sd.InputStream(samplerate=samplerate, channels=channels, dtype="float32", callback=callback):
        while not state.stop_event.is_set():
            time.sleep(0.05)

    state.recording = False
    if not frames:
        return None

    audio = np.concatenate(frames, axis=0)
    if channels == 1:
        audio = audio[:, 0]
    return audio


def transcribe_audio(client_llm, audio_np, samplerate: int, retries: int = 2, retry_delay: float = 1.0) -> str:
    import soundfile as sf

    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
        wav_path = tmp.name

    try:
        sf.write(wav_path, audio_np, samplerate)
        last_exc = None
        for attempt in range(1, max(1, retries) + 1):
            try:
                with open(wav_path, "rb") as f:
                    tr = client_llm.audio.transcriptions.create(
                        model=get_stt_model(),
                        file=f,
                        language="ru",
                    )
                text = getattr(tr, "text", "")
                text = (text or "").strip()
                print(f"📝 Распознано: {text!r}")
                return text
            except Exception as exc:
                last_exc = exc
                print(f"⚠️ STT попытка {attempt}/{max(1, retries)}: {exc}")
                if attempt < max(1, retries):
                    time.sleep(max(0.0, retry_delay))

        raise RuntimeError(f"STT failed after {max(1, retries)} attempts: {last_exc}")
    finally:
        try:
            os.remove(wav_path)
        except Exception:
            pass


def fallback_action_from_text(text: str):
    t = text.lower()
    if not t:
        return {"action": "noop"}
    if any(x in t for x in ["откр", "разож", "release", "reset"]):
        return {"action": "open"}
    if any(x in t for x in ["кулак", "сжми", "зажми", "grasp", "схвати"]):
        return {"action": "grasp"}
    if "ок" in t or "ok" in t:
        return {"action": "pose", "pose": "7"}
    if "рок" in t:
        return {"action": "pose", "pose": "4"}
    if "побед" in t or "мир" in t or "victory" in t:
        return {"action": "pose", "pose": "3"}
    m = re.search(r"\b([1-7])\b", t)
    if m:
        return {"action": "pose", "pose": m.group(1)}
    if any(x in t for x in ["выход", "стоп", "quit", "exit"]):
        return {"action": "exit"}
    return {"action": "noop"}


def print_available_commands() -> None:
    print("Доступные команды:")
    print("  open / открыть / разжать / reset")
    print("  grasp / кулак / сжать / схватить")
    print("  pose 1..7  (или просто цифра 1..7)")
    print("  ok / рок / победа / мир")
    print("  exit / выход")


def print_pose_mapping() -> None:
    print("Соответствие цифр:")
    for k in ["1", "2", "3", "4", "5", "6", "7"]:
        print(f"  {k} -> {POSES[k]['name']}")


def resolve_action_from_text(text: str, llm_client=None):
    if llm_client is None:
        action = fallback_action_from_text(text)
        print(f"🤖 local parser: {action}")
        return action
    return llm_to_action(llm_client, text)


def llm_to_action(client_llm, text: str):
    if not text:
        return {"action": "noop"}

    system_prompt = (
        "Ты управляешь роботизированной рукой. Верни только JSON без пояснений. "
        "Формат: {\"action\":\"pose|open|grasp|exit|noop\", \"pose\":\"1..7\"}. "
        "Если action не pose, поле pose можно опустить."
    )

    resp = client_llm.chat.completions.create(
        model=get_chat_model(),
        temperature=0,
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": text},
        ],
    )
    content = (resp.choices[0].message.content or "").strip()
    print(f"🤖 LLM: {content}")

    try:
        return json.loads(content)
    except Exception:
        m = re.search(r"\{.*\}", content, flags=re.S)
        if m:
            try:
                return json.loads(m.group(0))
            except Exception:
                pass
    return fallback_action_from_text(text)


def execute_action(hand_client: RH56DFTP_TCP, action_obj) -> bool:
    action = str(action_obj.get("action", "noop")).lower()

    if action == "pose":
        pose = str(action_obj.get("pose", ""))
        if pose in POSES:
            beep()
            execute_pose(hand_client, pose)
        else:
            print(f"❌ Невалидный номер позы: {pose}")
        return True

    if action == "open":
        beep()
        reset_pose(hand_client)
        return True

    if action == "grasp":
        beep()
        execute_pose(hand_client, "2")
        return True

    if action == "exit":
        print("🛑 Команда выхода")
        return False

    print("ℹ️ Ничего не выполняю (noop)")
    return True


def parse_args():
    p = argparse.ArgumentParser(description="Voice control for Inspire Hand via Shift+Space / Space")
    p.add_argument("--ip", default="192.168.123.211", help="Hand IP")
    p.add_argument("--port", type=int, default=6000, help="Hand port")
    p.add_argument(
        "--mode",
        choices=["voice", "text"],
        default="voice",
        help="Control mode: voice hotkeys or text commands",
    )
    p.add_argument("--samplerate", type=int, default=16000, help="Mic sample rate")
    p.add_argument("--channels", type=int, default=1, help="Mic channels")
    p.add_argument("--stt-retries", type=int, default=3, help="Retry count for STT request")
    p.add_argument("--stt-retry-delay", type=float, default=1.5, help="Delay between STT retries (sec)")
    p.add_argument(
        "--text-cmd",
        default="",
        help="Execute one text command and exit (e.g. 'pose 7' or 'open')",
    )
    p.add_argument(
        "--no-llm",
        action="store_true",
        help="Disable LLM mapping and use local text parser",
    )
    p.add_argument(
        "--sdk-log-level",
        default="WARNING",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
        help="Log level for RH56DFTP/pymodbus internals",
    )
    p.add_argument(
        "--skip-init-open",
        action="store_true",
        help="Do not send initial open-pose command on startup",
    )
    return p.parse_args()


def main():
    args = parse_args()
    lvl = getattr(logging, args.sdk_log_level.upper(), logging.WARNING)
    logging.getLogger("RH56DFTP").setLevel(lvl)
    logging.getLogger("pymodbus").setLevel(lvl)
    logging.getLogger("httpx").setLevel(logging.WARNING)
    logging.getLogger("openai").setLevel(logging.WARNING)

    print("🔌 Подключение к руке...")
    hand_client = RH56DFTP_TCP(host=args.ip, port=args.port)
    print(f"✅ Рука: {args.ip}:{args.port}")

    llm_client = None
    if not args.no_llm:
        print("🔌 Подключение к LLM/STT...")
        try:
            llm_client = get_vsellm_client()
            print(f"✅ LLM model: {get_chat_model()} | STT model: {get_stt_model()}")
        except Exception as exc:
            print(f"⚠️ LLM недоступен: {exc}")
            if args.mode == "voice":
                print("⚠️ Для voice-режима нужен LLM/STT. Переключаюсь в text + local parser.")
                args.mode = "text"
            args.no_llm = True
    else:
        print("ℹ️ LLM отключен (--no-llm). Используется локальный парсер команд.")

    try:
        if not args.skip_init_open:
            print("ℹ️ Инициализация: открываю ладонь...", flush=True)
            reset_pose(hand_client)
            print("ℹ️ Инициализация завершена.", flush=True)

        if args.mode == "text":
            print("")
            print_available_commands()

            if args.text_cmd:
                action_obj = resolve_action_from_text(args.text_cmd, llm_client=llm_client)
                execute_action(hand_client, action_obj)
                return

            while True:
                raw = input("cmd> ").strip()
                if not raw:
                    continue
                if raw.lower() in {"help", "h", "?", "команды"}:
                    print_available_commands()
                    continue
                action_obj = resolve_action_from_text(raw, llm_client=llm_client)
                if not execute_action(hand_client, action_obj):
                    break
            return

        state = HotkeyState()
        listener = make_keyboard_listener(state)
        listener.start()

        print("\nГорячие клавиши:")
        if state.mode == "pynput":
            print("  Shift+Space  -> старт записи")
            print("  Space        -> стоп записи и выполнить команду")
        else:
            print("  S            -> старт записи (fallback без X)")
            print("  Space        -> стоп записи и выполнить команду")
        print("  1..7         -> сразу выполнить позу")
        print("  R            -> открыть ладонь")
        print("  G            -> кулак")
        print("  X            -> выход")
        print("  Ctrl+C       -> выход")
        if llm_client is None:
            print("  [voice disabled] LLM/STT недоступен или выключен (--no-llm)")
        print("")
        print_pose_mapping()
        print("", flush=True)

        while True:
            # Priority 1: immediate typed command (1..7/R/G/X).
            try:
                cmd = state.command_queue.get_nowait()
                print(f"⌨️ Команда: {cmd}")
                # Keyboard shortcuts are always local (no LLM token usage).
                action_obj = resolve_action_from_text(cmd, llm_client=None)
                if not execute_action(hand_client, action_obj):
                    break
                continue
            except queue.Empty:
                pass

            # Priority 2: voice flow.
            if not state.start_event.wait(timeout=0.1):
                continue
            state.start_event.clear()
            if llm_client is None:
                print("⚠️ Голосовой ввод отключен без LLM/STT. Используйте 1..7/R/G/X.")
                continue

            audio_np = record_audio_until_space(state, samplerate=args.samplerate, channels=args.channels)
            if audio_np is None or len(audio_np) == 0:
                print("⚠️ Пустая запись")
                continue

            try:
                text = transcribe_audio(
                    llm_client,
                    audio_np,
                    args.samplerate,
                    retries=args.stt_retries,
                    retry_delay=args.stt_retry_delay,
                )
            except Exception as exc:
                print(f"❌ Ошибка распознавания: {exc}")
                print("ℹ️ Проверьте сеть/VPN до api.vsellm.ru и попробуйте снова.")
                continue
            action_obj = resolve_action_from_text(text, llm_client=llm_client)
            if not execute_action(hand_client, action_obj):
                break

    except KeyboardInterrupt:
        print("\n👋 Остановлено пользователем")
    finally:
        try:
            listener.stop()
        except Exception:
            pass
        try:
            hand_client.close()
        except Exception:
            pass
        print("🔌 Соединение закрыто")


if __name__ == "__main__":
    main()
