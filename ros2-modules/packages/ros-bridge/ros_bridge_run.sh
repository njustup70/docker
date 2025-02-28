#!/bin/bash

# 使脚本支持 Ctrl+C 快速退出
trap 'echo "Exiting..."; kill $(jobs -p)' SIGINT
# 获取当前脚本所在目录
SCRIPT_DIR=$(dirname "$(realpath "$0")")

# 选择架构并配置环境
source ~/.bashrc
if [ `uname -m` == "x86_64" ]; then
    # x86
    echo "x86"
    source $SCRIPT_DIR/x86/setup.bash
else
    # arm
    echo "arm"
    source $SCRIPT_DIR/arm/setup.bash
fi

# 启动 ros2 动态桥接，并将其放入后台
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics &

# 等待直到脚本接收到 Ctrl+C
wait
