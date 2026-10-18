#!/usr/bin/env python3
"""Проверка Python, SDK2, сети и ROS без обращения к управлению G1"""

import argparse
import importlib
import importlib.metadata
import json
import os
import platform
import socket
import sys
from pathlib import Path


def os_info():
    result = {}
    path = Path("/etc/os-release")
    if path.exists():
        for line in path.read_text(encoding="utf-8").splitlines():
            if "=" in line and not line.startswith("#"):
                k, v = line.split("=", 1)
                result[k] = v.strip().strip('"')
    return result


def check():
    ap = argparse.ArgumentParser(description="Проверка окружения WearBench для Ubuntu 20.04/22.04")
    ap.add_argument("--iface", default="eth0", help="Сетевой интерфейс для ChannelFactoryInitialize")
    ap.add_argument("--require-ros", action="store_true",
                    help="Проверить наличие Foxy/Humble; регистратору ROS не нужен")
    args = ap.parse_args()
    dist = os_info()
    py = "{}.{}.{}".format(*sys.version_info[:3])
    iface_list = [name for _, name in socket.if_nameindex()]
    ros = {d: Path("/opt/ros/{}/setup.bash".format(d)).is_file()
           for d in ("foxy", "humble")}
    recommended = "foxy" if dist.get("VERSION_ID", "").startswith("20.04") else (
        "humble" if dist.get("VERSION_ID", "").startswith("22.04") else None)
    report = {
        "platform": platform.system(),
        "arch": platform.machine(),
        "os": dist.get("PRETTY_NAME", platform.platform()),
        "python": py,
        "python_executable": sys.executable,
        "interfaces": iface_list,
        "selected_iface": args.iface,
        "ros_installed": ros,
        "recommended_ros": recommended,
        "active_ros_distro": os.environ.get("ROS_DISTRO"),
        "active_rmw": os.environ.get("RMW_IMPLEMENTATION"),
        "sdk_source": None,
        "cyclonedds_version": None,
        "sdk_import": None,
        "warnings": [],
        "errors": [],
    }
    if sys.version_info < (3, 8):
        report["errors"].append("Нужен Python >=3.8")
    if sys.version_info[:2] not in ((3, 8), (3, 10)):
        report["warnings"].append("Ожидался системный Python 3.8 или 3.10")
    if args.iface not in iface_list:
        report["errors"].append("Интерфейс {} не найден".format(args.iface))
    if args.require_ros and not any(ros.values()):
        report["errors"].append("ROS Foxy/Humble не обнаружен")
    if recommended and not ros.get(recommended, False):
        report["warnings"].append("ROS {} не найден; для DDS регистратора ROS не требуется".format(recommended))
    if recommended and os.environ.get("ROS_DISTRO") not in (None, "", recommended):
        report["warnings"].append("Активный ROS_DISTRO не соответствует этой версии Ubuntu")
    if sum(ros.values()) > 1:
        report["warnings"].append("Найдены Foxy и Humble; автоопределение старого JSON launcher сначала выбирает Humble")

    sdk_path = Path(__file__).resolve().parents[1] / "unitree_sdk2_python_custom"
    if sdk_path.is_dir():
        sys.path.insert(0, str(sdk_path))
        report["sdk_source"] = str(sdk_path)
    else:
        report["sdk_source"] = "installed"
    try:
        report["cyclonedds_version"] = importlib.metadata.version("cyclonedds")
    except importlib.metadata.PackageNotFoundError:
        report["warnings"].append("CycloneDDS Python не установлен в текущем интерпретаторе")
    if report["cyclonedds_version"] and report["cyclonedds_version"] != "0.10.2":
        report["warnings"].append("SDK в этой ветке использует cyclonedds==0.10.2, проверьте фактическую совместимость")
    try:
        importlib.import_module("cyclonedds.idl")
        channel = importlib.import_module("unitree_sdk2py.core.channel")
        messages = importlib.import_module("unitree_sdk2py.idl.unitree_hg.msg.dds_")
        assert getattr(channel, "ChannelSubscriber", None) is not None
        assert getattr(channel, "ChannelFactoryInitialize", None) is not None
        assert getattr(messages, "LowState_", None) is not None
        report["sdk_import"] = "OK"
    except Exception as exc:
        report["sdk_import"] = repr(exc)
        report["errors"].append("Не удалось импортировать DDS/SDK2 в этом Python")

    print(json.dumps(report, indent=2, ensure_ascii=False))
    if report["errors"]:
        print("PREFLIGHT FAILED: ничего в окружении не менялось")
        return 2
    print("PREFLIGHT OK: импорты и интерфейс есть, реальное получение DDS ещё не проверено")
    return 0


if __name__ == "__main__":
    sys.exit(check())
