#!/bin/bash

# 等待挂载目录准备好
until [ -d "/home/Elaina/docker/rosbridge" ]; do
    echo "Waiting for the mount to be ready..."
    sleep 1
done

echo "Mount ready, starting ros2 bridge..."
source ~/.bashrc
source /home/Elaina/docker/rosbridge/ros-humble-ros1-bridge/install/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics