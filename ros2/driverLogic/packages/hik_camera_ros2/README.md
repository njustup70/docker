# 海康威视（HIKROBOT）工业相机 ROS 2 驱动程序 (hik_camera_ros2)

[![ROS Version](https://img.shields.io/badge/ROS-Humble-blue)](https://docs.ros.org/en/humble/index.html)

这是一个海康威视（HIKROBOT）工业相机开发的 ROS 2 Humble 功能包，旨在提供一个稳定、易于配置且功能完善的相机驱动节点。线下验收时实时帧率最高跑到了164。
---

## ⚙️ 依赖项和安装

### 1. 环境需求
-   Ubuntu 22.04
-   ROS 2 Humble Hawksbill
-   机器视觉工业相机客户端MVS V4.6.0（Linux）

### 2. ROS2配置、编译和运行 
```zsh
# 进入您的 ROS 2 工作空间根目录
cd ~/ros2_ws

# 编译本功能包
colcon build --packages-select hik_camera_ros2

# ...
parameters=[
    # 將 'YOUR_CAMERA_SERIAL_NUMBER' 替换为真实序列号
    {'serial_number': 'YOUR_CAMERA_SERIAL_NUMBER'}, 
    
    # 自定义话题名称
    {'topic_name': '/hik_camera/image_raw'}, 
    
    # 参数设定
    {'exposure_time': 50000.0},  # 单位: 微秒 (µs)
    {'gain': 8.0},
    {'frame_rate': 10.0},        # 目标帧率
    {'pixel_format': 'BayerRG8'},  # "Mono8", "BayerRG8" 等
],
# ...

# 启用工作空间的环境
source install/setup.zsh

# 执行 Launch 文件
ros2 launch hik_camera_ros2 hik_camera.launch.py
