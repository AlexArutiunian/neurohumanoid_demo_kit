#!/usr/bin/env python3
"""Helpers to build OpenAI-compatible client for VseLLM from env."""

import os
from pathlib import Path

import httpx
import openai


def _load_env_files() -> None:
    """Load env vars from /env and .env if python-dotenv is installed."""
    try:
        from dotenv import load_dotenv  # type: ignore
    except Exception:
        return

    # The user explicitly mentioned "/env".
    load_dotenv("/env", override=False)
    # Current working directory.
    load_dotenv(Path.cwd() / ".env", override=False)
    # .env next to this module (DFTP_arm/.env).
    module_dir = Path(__file__).resolve().parent
    load_dotenv(module_dir / ".env", override=False)
    # Project root .env (../.env from this file).
    load_dotenv(module_dir.parent / ".env", override=False)


def get_vsellm_client() -> openai.OpenAI:
    _load_env_files()

    api_key = (
        os.getenv("VSELLM_API_KEY")
        or os.getenv("OPENAI_API_KEY")
        or os.getenv("API_KEY")
    )
    if not api_key:
        raise RuntimeError(
            "API key not found. Set VSELLM_API_KEY in /env or .env"
        )

    base_url = os.getenv("VSELLM_BASE_URL", "https://api.vsellm.ru/v1")
    connect_timeout = float(os.getenv("VSELLM_CONNECT_TIMEOUT_SEC", "30"))
    read_timeout = float(os.getenv("VSELLM_READ_TIMEOUT_SEC", "180"))
    write_timeout = float(os.getenv("VSELLM_WRITE_TIMEOUT_SEC", "180"))
    pool_timeout = float(os.getenv("VSELLM_POOL_TIMEOUT_SEC", "30"))
    timeout = httpx.Timeout(
        connect=connect_timeout,
        read=read_timeout,
        write=write_timeout,
        pool=pool_timeout,
    )
    return openai.OpenAI(api_key=api_key, base_url=base_url, timeout=timeout)


def get_chat_model() -> str:
    _load_env_files()
    return os.getenv("VSELLM_CHAT_MODEL", "anthropic/claude-haiku-4.5")


def get_stt_model() -> str:
    _load_env_files()
    return os.getenv("VSELLM_STT_MODEL", "openai/whisper-large-v3")


if __name__ == "__main__":
    client = get_vsellm_client()
    response = client.chat.completions.create(
        model=get_chat_model(),
        messages=[{"role": "user", "content": "Привет!"}],
    )
    print(response.choices[0].message.content)
