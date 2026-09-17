#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shlex
import subprocess
from pathlib import Path


def normalize(text: str) -> str:
    return " ".join(text.lower().replace("ё", "е").strip().split())


def load_cfg(path: str) -> dict:
    with open(Path(path).expanduser(), "r", encoding="utf-8") as f:
        return json.load(f)


def resolve_motion_key(command: str, cfg: dict) -> str | None:
    cmd = normalize(command)
    motions = cfg.get("motions", {})

    if cmd in motions:
        return cmd

    aliases = cfg.get("aliases", {})
    for key, phrases in aliases.items():
        for p in phrases:
            p_norm = normalize(str(p))
            if p_norm == cmd or (p_norm and p_norm in cmd):
                return key

    return None


def run_play_command(play_cmd_template: str, json_path: str) -> tuple[int, str]:
    cmd = play_cmd_template.format(json_path=shlex.quote(json_path))
    proc = subprocess.run(
        ["bash", "-lc", cmd],
        capture_output=True,
        text=True,
        check=False,
    )
    output = (proc.stdout or "").strip()
    err = (proc.stderr or "").strip()
    if err:
        output = f"{output}\n{err}".strip()
    return proc.returncode, output


def main():
    parser = argparse.ArgumentParser(description="Map voice command to motion JSON and play it")
    parser.add_argument("--command", required=True, help="Recognized voice command text")
    parser.add_argument("--config", required=True, help="Path to robot_motion_map.json")
    args = parser.parse_args()

    cfg = load_cfg(args.config)
    key = resolve_motion_key(args.command, cfg)
    if key is None:
        print(f"NO_MATCH command='{args.command}'")
        return

    motion = cfg["motions"].get(key, {})
    json_path = str(Path(motion.get("json_path", "")).expanduser())
    if not json_path:
        print(f"BAD_CONFIG missing json_path for key='{key}'")
        return
    if not Path(json_path).exists():
        print(f"MISSING_JSON key='{key}' path='{json_path}'")
        return

    play_cmd_template = cfg.get("play_cmd_template", "").strip()
    if not play_cmd_template:
        print("BAD_CONFIG missing play_cmd_template")
        return

    code, out = run_play_command(play_cmd_template, json_path)
    if code == 0:
        print(f"OK key='{key}' json='{json_path}'")
    else:
        print(f"PLAY_ERROR key='{key}' json='{json_path}' code={code}")
    if out:
        print(out)


if __name__ == "__main__":
    main()
