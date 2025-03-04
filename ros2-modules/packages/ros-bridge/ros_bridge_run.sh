#!/bin/bash

# 使脚本支持 Ctrl+C 快速退出
trap 'echo "Exiting..."; kill $(jobs -p)' SIGINT
# 获取当前脚本所在目录
SCRIPT_DIR=$(dirname "$(realpath "$0")")


# 检查端口 11311 是否被占用
check_port() {
    local port=11311
    local result=$(sudo netstat -tulnp | grep ":$port")

    if [[ -n "$result" ]]; then
        if echo "$result" | grep -q "python3"; then
            echo -e "\033[1;32m端口 $port 被 python3 进程占用，继续执行。\033[0m"
        else
            echo -e "\033[1;31m错误: 端口 $port 被非 python3 进程占用！\033[0m"
            echo -e "\033[1;31m请检查后再运行此脚本。\033[0m"
            exit 1
        fi
    fi
}

check_port || exit 1
# 选择架构并配置环境
source ~/.bashrc
if [ `uname -m` == "x86_64" ]; then
    # x86
    echo "x86"
    source $SCRIPT_DIR/x86/local_setup.bash
else
    # arm
    echo "arm"
    source $SCRIPT_DIR/arm/local_setup.bash
fi

# 启动 ros2 动态桥接，并将其放入后台
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics &

# 等待直到脚本接收到 Ctrl+C
wait
