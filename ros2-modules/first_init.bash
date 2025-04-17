#!/bin/bash

# 获取脚本的绝对路径
SCRIPT_DIR=$(dirname "$(realpath "$0")")

# 进入脚本同目录下的 librealsense 目录
cd "$SCRIPT_DIR/packages/librealsense"
echo "当前目录: $(pwd)"
./scripts/setup_udev_rules.sh

# 安装奥比中光规则
cd "$SCRIPT_DIR/packages/orbbecSDK/misc/scripts"
echo "当前目录: $(pwd)"
sudo ./install_udev_rules.sh

# 安装 wheel_imu 规则
cd "$SCRIPT_DIR/packages/wheel_imu/fdilink_ahrs_ROS2"
echo "当前目录: $(pwd)"
sudo ./wheeltec_udev.sh
