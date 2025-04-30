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

#安装ms_200规则
# 设置设备别名并设置权限
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", MODE:="0777", GROUP:="dialout", SYMLINK+="ms200"' > /etc/udev/rules.d/my_dev.rules
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", MODE:="0777", GROUP:="dialout", SYMLINK+="ms200"' >> /etc/udev/rules.d/my_dev.rules

# 下位机串口
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0777", GROUP:="dialout", SYMLINK+="serial_x64"' >> /etc/udev/rules.d/my_dev.rules
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0777", GROUP:="dialout", SYMLINK+="serial_x64"' >> /etc/udev/rules.d/my_dev.rules


service udev reload
sleep 2
service udev restart


