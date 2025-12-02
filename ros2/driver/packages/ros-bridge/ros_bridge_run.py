#!/usr/bin/env python3
import os
import signal
import subprocess
import sys
import platform
from pathlib import Path
import shutil

# Ctrl+C 快速退出
def signal_handler(sig, frame):
    print("Exiting...")
    if bridge_process and bridge_process.poll() is None:
        bridge_process.terminate()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# 获取当前脚本所在目录
SCRIPT_DIR = Path(__file__).resolve().parent

# 检查端口 11311 是否被占用
def check_port():
    port = "11311"
    try:
        result = subprocess.check_output(
            ["sudo", "netstat", "-tulnp"],
            stderr=subprocess.STDOUT
        ).decode()
    except subprocess.CalledProcessError as e:
        print("无法检测端口:", e.output.decode(), file=sys.stderr)
        return False

    lines = [line for line in result.splitlines() if f":{port}" in line]
    if lines:
        if any("python3" in line for line in lines):
            print(f"\033[1;32m端口 {port} 被 python3 进程占用，继续执行。\033[0m")
        else:
            print(f"\033[1;31m错误: 端口 {port} 被非 python3 进程占用！\033[0m")
            print(f"\033[1;31m请检查后再运行此脚本。\033[0m")
            return False
    return True

if not check_port():
    sys.exit(1)

# Source ~/.bashrc（实际上不能直接用 Python 完全替代 bash 的 source，这里我们只打印提醒）
# subprocess.call(["bash", "-c", "source ~/.bashrc"])

# 判断架构
arch = platform.machine()
if arch == "x86_64":
    print("x86")
    setup_script = SCRIPT_DIR / "x86" / "local_setup.bash"
else:
    print("arm")
    setup_script = SCRIPT_DIR / "arm" / "local_setup.bash"

# source local_setup.bash 并运行 dynamic_bridge
# 为了让环境变量生效，我们在同一个 bash shell 中运行 ros2
if check_port():
    bridge_process = subprocess.Popen(
        f"bash -c 'source {setup_script} && ros2 run ros1_bridge dynamic_bridge --bridge-all-topics'",
        shell=True,
        executable="/bin/bash",
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

# 等待 Ctrl+C
bridge_process.wait()
