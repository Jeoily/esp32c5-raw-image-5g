"""Local launcher: builds only on request; flashes only on an explicit button click."""
import argparse
import json
import os
from pathlib import Path
import queue
import re
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

from serial.tools import list_ports

ROOT = Path(__file__).resolve().parents[1]
SETTINGS = ROOT / "demo_settings.json"
BAUD = 921600
HIDDEN = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0


def load_settings():
    return json.loads(SETTINGS.read_text(encoding="utf-8-sig"))


def build_environment(settings):
    """Use the installed C5 toolchain, without changing the SDK installation."""
    env = os.environ.copy()
    idf = Path(settings["idf_path"])
    tools = Path(settings["idf_tools_path"])
    python = Path(settings["idf_python"])
    if not python.is_file() or not (idf / "tools" / "idf.py").is_file():
        raise RuntimeError("找不到 ESP-IDF 或其 Python，请检查 demo_settings.json 中的路径。")
    directories = [python.parent, idf / "tools"]
    for pattern in ("riscv32-esp-elf/*/riscv32-esp-elf/bin", "cmake/*/bin", "ninja/*", "idf-exe/*"):
        matches = sorted(p for p in tools.glob(pattern) if p.is_dir())
        if not matches:
            raise RuntimeError(f"缺少已安装的构建工具：{pattern}。请先在 ESP-IDF 扩展中完成环境安装。")
        directories.append(matches[-1])
    env.update(IDF_PATH=str(idf), IDF_TOOLS_PATH=settings["idf_tools_path"],
               IDF_PYTHON_ENV_PATH=str(python.parent.parent), IDF_TARGET="esp32c5",
               PYTHONUTF8="1", PYTHONIOENCODING="utf-8")
    env["IDF_COMPONENT_LOCAL_STORAGE_URL"] = tools.as_uri()
    version_text = (idf / "tools" / "cmake" / "version.cmake").read_text(encoding="utf-8")
    parts = []
    for name in ("MAJOR", "MINOR", "PATCH"):
        match = re.search(r"set\(IDF_VERSION_" + name + r"\s+(\d+)\)", version_text)
        if not match:
            raise RuntimeError("无法读取 ESP-IDF 版本，请核对配置的SDK路径。")
        parts.append(match.group(1))
    env["ESP_IDF_VERSION"] = ".".join(parts)
    env["PATH"] = os.pathsep.join(map(str, directories)) + os.pathsep + env.get("PATH", "")
    rom = sorted(p for p in tools.glob("esp-rom-elfs/*") if p.is_dir())
    if rom:
        env["ESP_ROM_ELF_DIR"] = str(rom[-1])
    return env


def build_command(settings, role, port=None):
    command = [settings["idf_python"], str(Path(settings["idf_path"]) / "tools" / "idf.py"),
               "-C", str(ROOT / role), "-B", str(ROOT / role / "build_c5")]
    # Explicit C5 configuration avoids both the copied build cache and old S3 defaults.
    command += ["-D", "IDF_TARGET=esp32c5"]
    if port is None:
        command.append("build")
    else:
        if not re.fullmatch(r"COM[1-9][0-9]*", port, re.IGNORECASE):
            raise ValueError("请选择实际的 COM 串口。")
        command += ["-p", port, "flash"]
    return command


def run_build(settings, role, port=None, log=print):
    cache = ROOT / role / 'build_c5' / 'CMakeCache.txt'
    if cache.exists():
        cached = cache.read_text(encoding='utf-8', errors='replace')
        match = re.search(r'^CMAKE_HOME_DIRECTORY:INTERNAL=(.+)$', cached, re.MULTILINE)
        if match and Path(match.group(1).strip()).resolve() != (ROOT / role).resolve():
            backup = cache.parent.with_name('build_c5_backup_' + time.strftime('%Y%m%d_%H%M%S'))
            cache.parent.rename(backup)
            log(f'旧路径编译缓存已保留到 {backup}，将重新构建。')
    log(f"\n开始：{role} {'编译并烧录到 ' + port if port else '仅编译，不访问开发板'}")
    log("首次编译需要等待，日志会持续更新。")
    logs = ROOT / "launcher_logs"
    logs.mkdir(exist_ok=True)
    name = f"{role}_{'flash' if port else 'build'}_{time.strftime('%Y%m%d_%H%M%S')}.log"
    with (logs / name).open("w", encoding="utf-8") as output:
        process = subprocess.Popen(build_command(settings, role, port), env=build_environment(settings),
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   text=True, encoding="utf-8", errors="replace", creationflags=HIDDEN)
        for line in process.stdout:
            output.write(line)
            output.flush()
            log(line.rstrip())
        code = process.wait()
    if code:
        raise RuntimeError(f"{role} 执行失败（退出码 {code}）。完整日志：{logs / name}")
    log(f"{role} 完成。日志：{logs / name}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Only compile; never flash or open serial ports.')
    parser.add_argument('--role', choices=('Alice', 'Bob', 'both'), default='both')
    args = parser.parse_args()
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    for role in (('Bob', 'Alice') if args.role == 'both' else (args.role,)):
        run_build(load_settings(), role)



